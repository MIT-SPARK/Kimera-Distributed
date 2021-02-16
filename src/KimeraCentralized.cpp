/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */
#define SLOW_BUT_CORRECT_BETWEENFACTOR
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <nav_msgs/Path.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphNode.h>
#include <pose_graph_tools/PoseGraphQuery.h>
#include <pose_graph_tools/utils.h>

#include <kimera_distributed/prefix.h>

#include "kimera_distributed/KimeraCentralized.h"

namespace kimera_distributed {

KimeraCentralized::KimeraCentralized(const ros::NodeHandle& n) : nh_(n) {
  // Get parameters
  b_log_output_ = ros::param::get("~log_output_path", log_output_path_);

  if (!ros::param::get("~num_robots", num_robots_)) {
    ROS_ERROR("Failed to get the number of robots! Shutting down... ");
    ros::shutdown();
  }

  // Initialize optimized_path
  for (size_t i = 0; i < num_robots_; i++) {
    optimized_path_.push_back(std::vector<gtsam::Pose3>{});
  }

  // Covariance parameters
  double odom_trans_precision, odom_rot_precision;
  double lc_trans_precision, lc_rot_precision;
  if (!ros::param::get("~odom_trans_precision", odom_trans_precision) ||
      !ros::param::get("~odom_rot_precision", odom_rot_precision) ||
      !ros::param::get("~lc_trans_precision", lc_trans_precision) ||
      !ros::param::get("~lc_rot_precision", lc_rot_precision)) {
    ROS_ERROR(
        "Failed to get odom and loop closure precisions! Shutting down... ");
    ros::shutdown();
  }

  // Currently fixing covariance from params
  gtsam::Vector6 odom_precisions, lc_precisions;
  odom_precisions.head<3>().setConstant(odom_rot_precision);
  odom_precisions.tail<3>().setConstant(odom_trans_precision);
  odom_noise_ = gtsam::noiseModel::Diagonal::Precisions(odom_precisions);
  lc_precisions.head<3>().setConstant(lc_rot_precision);
  lc_precisions.tail<3>().setConstant(lc_trans_precision);
  loop_closure_noise_ = gtsam::noiseModel::Diagonal::Precisions(lc_precisions);

  // Pcm parameters
  double pcm_trans_threshold, pcm_rot_threshold;
  double gnc_alpha;
  if (!ros::param::get("~pcm_threshold_translation", pcm_trans_threshold) ||
      !ros::param::get("~pcm_threshold_rotation", pcm_rot_threshold) ||
      !ros::param::get("~gnc_confidence", gnc_alpha)) {
    ROS_ERROR("Failed to get required PCM parameters! Shutting down... ");
    ros::shutdown();
  }

  // Initialize pcm
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(pcm_trans_threshold, pcm_rot_threshold);
  pgo_params.logOutput(log_output_path_);
  pgo_params.setIncremental();
  pgo_params.setGncInlierCostThresholdsAtProbability(gnc_alpha);
  pgo_ = std::unique_ptr<KimeraRPGO::RobustSolver>(
      new KimeraRPGO::RobustSolver(pgo_params));

  // Initialize publishers
  pose_graph_pub_ = nh_.advertise<pose_graph_tools::PoseGraph>(
      "pose_graph_optimized", 1, false);

  for (size_t i = 0; i < num_robots_; i++) {
    std::string path_topic = "robot_" + std::to_string(i) + "_optimized_path";
    path_pub_.push_back(nh_.advertise<nav_msgs::Path>(path_topic, 1, false));
  }

  // Initialize timer
  update_timer_ = nh_.createTimer(
      ros::Duration(1.0), &KimeraCentralized::timerCallback, this);
}

KimeraCentralized::~KimeraCentralized() {}

void KimeraCentralized::timerCallback(const ros::TimerEvent&) {
  // Request pose grah
  gtsam::NonlinearFactorGraph nfg_new;
  gtsam::Values values_new;
  getNewPoseGraph(&nfg_new, &values_new);

  // Add and update rpgo
  pgo_->update(nfg_new, values_new);
  // Get the new factor graph and estimates
  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();
  // Update optimized path
  updateOptimizedPath();

  // Publish and log
  publishPoseGraph();
  for (size_t i = 0; i < num_robots_; i++) {
    publishRobotTrajectory(i);

    if (b_log_output_) {
      std::string filename =
          log_output_path_ + "robot_" + std::to_string(i) + "_traj.csv";
      logRobotTrajectory(i, filename);
    }
  }
}

void KimeraCentralized::updateOptimizedPath() {
  // Use latest estimates to update optimized path
  // values_.keys() should be ordered
  for (const auto& key : values_.keys()) {
    gtsam::Symbol key_symb(key);
    size_t robot_id = robot_prefix_to_id.at(key_symb.chr());
    size_t index = key_symb.index();
    assert(robot_id < num_robots_ && robot_id >= 0);
    if (index < optimized_path_[robot_id].size()) {
      optimized_path_[robot_id][index] = values_.at<gtsam::Pose3>(key);
    } else {
      optimized_path_[robot_id].push_back(values_.at<gtsam::Pose3>(key));
    }
  }
}

void KimeraCentralized::getNewPoseGraph(
    gtsam::NonlinearFactorGraph* new_factors,
    gtsam::Values* new_values) const {
  assert(NULL != new_factors);
  assert(NULL != new_values);
  // Request the local pose graph from the robots
  // Note: remove duplicated shared loop closures
  for (size_t i = 0; i < num_robots_; i++) {
    // odometry factors
    gtsam::NonlinearFactorGraph odom_meas;
    // single robot loop closure factors
    gtsam::NonlinearFactorGraph lc_meas;
    // Inter-robot loop closure factors
    gtsam::NonlinearFactorGraph inter_lc_meas;
    // Initial estimates
    gtsam::Values init_values;
    // Get the factors and initial estimates for robot i
    requestRobotPoseGraph(
        i, &odom_meas, &lc_meas, &inter_lc_meas, &init_values);

    // Add the new odometry factors
    new_factors->add(odom_meas);

    // Add the new loop closures
    new_factors->add(lc_meas);

    // Add new shared loop closures and check for duplicates with other
    // robots
    for (const auto& shared_lc_factor : inter_lc_meas) {
      // Check for duplicates
      bool duplicate = false;
      for (const auto& new_factor : *new_factors) {
        if (new_factor->front() == shared_lc_factor->front() &&
            new_factor->back() == shared_lc_factor->back()) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) new_factors->push_back(shared_lc_factor);
    }

    // Add the new initial estimates
    new_values->insert(init_values);
  }
}

bool KimeraCentralized::requestRobotPoseGraph(
    const size_t& robot_id,
    gtsam::NonlinearFactorGraph* odom_factors,
    gtsam::NonlinearFactorGraph* private_lc_factors,
    gtsam::NonlinearFactorGraph* shared_lc_factors,
    gtsam::Values* values) const {
  // Request pose graph and find the new nodes and edges
  assert(NULL != odom_factors);
  assert(NULL != private_lc_factors);
  assert(NULL != shared_lc_factors);
  assert(NULL != values);
  // Request pose graph
  pose_graph_tools::PoseGraphQuery query;
  query.request.robot_id = robot_id;
  std::string service_name = "/kimera" + std::to_string(robot_id) +
                             "/distributed_pcm/request_pose_graph";
  if (!ros::service::waitForService(service_name, ros::Duration(1.0))) {
    ROS_ERROR_STREAM("ROS service " << service_name << " does not exist!");
    return false;
  }
  if (!ros::service::call(service_name, query)) {
    ROS_ERROR_STREAM("Failed to call ROS service " << service_name);
    return false;
  }

  // Set pose graph
  pose_graph_tools::PoseGraph pose_graph = query.response.pose_graph;
  if (pose_graph.edges.empty()) {
    ROS_WARN("Received empty pose graph.");
    return false;
  }

  for (size_t i = 0; i < pose_graph.edges.size(); ++i) {
    const pose_graph_tools::PoseGraphEdge& edge = pose_graph.edges[i];
    assert(edge.robot_from < num_robots_ && edge.robot_from >= 0);
    assert(edge.robot_to < num_robots_ && edge.robot_to >= 0);
    // check if new
    // For now we assume that new edges have at least on previously
    // unseen key frame.
    // TODO: double check this
    if (edge.key_from < optimized_path_[edge.robot_from].size() &&
        edge.key_to < optimized_path_[edge.robot_to].size())
      continue;  // Previously seen edge

    // Extract key and transform if new edge
    const gtsam::Pose3& meas = RosPoseToGtsam(edge.pose);
    const gtsam::Symbol from_key(robot_id_to_prefix.at(edge.robot_from),
                                 edge.key_from);
    const gtsam::Symbol to_key(robot_id_to_prefix.at(edge.robot_to),
                               edge.key_to);

    if (edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
      // odometry factor
      odom_factors->add(gtsam::BetweenFactor<gtsam::Pose3>(
          from_key, to_key, meas, odom_noise_));
    } else if (edge.type == pose_graph_tools::PoseGraphEdge::LOOPCLOSE) {
      // loop closure
      if (edge.robot_from == edge.robot_to) {
        // single robot loop closure
        private_lc_factors->add(gtsam::BetweenFactor<gtsam::Pose3>(
            from_key, to_key, meas, loop_closure_noise_));
      } else {
        // multi robot loop closure
        shared_lc_factors->add(gtsam::BetweenFactor<gtsam::Pose3>(
            from_key, to_key, meas, loop_closure_noise_));
      }
    } else {
      ROS_ERROR(
          "KimeraCentralized: Encountered unsupported edge type when "
          "requesting pose graph.");
    }
  }

  for (size_t i = 0; i < pose_graph.nodes.size(); i++) {
    const pose_graph_tools::PoseGraphNode& node = pose_graph.nodes[i];
    if (node.robot_id != robot_id) continue;
    // Check if new
    if (node.key >= optimized_path_[node.robot_id].size()) {
      const gtsam::Pose3& pose = RosPoseToGtsam(node.pose);
      const gtsam::Symbol key(robot_id_to_prefix.at(node.robot_id), node.key);
      values->insert(key, pose);
    }
  }

  ROS_INFO_STREAM("Agent " << robot_id << " receives local pose graph with "
                           << odom_factors->size() << " new odometry edges and "
                           << private_lc_factors->size()
                           << " new private loop closures and "
                           << shared_lc_factors->size()
                           << " new shared loop closures.");

  return true;
}

void KimeraCentralized::publishPoseGraph() const {
  if (pose_graph_pub_.getNumSubscribers() == 0) return;
  pose_graph_tools::PoseGraph pose_graph_msg = GtsamGraphToRos(nfg_, values_);
  pose_graph_msg.header.frame_id = "world";
  pose_graph_msg.header.stamp = ros::Time::now();

  pose_graph_pub_.publish(pose_graph_msg);
}

void KimeraCentralized::publishRobotTrajectory(const size_t& robot_id) const {
  assert(robot_id < num_robots_);
  if (path_pub_[robot_id].getNumSubscribers() == 0) return;
  // Convert gtsam pose3 path to nav msg
  const nav_msgs::Path& path_msg =
      GtsamPoseTrajectoryToPath(optimized_path_[robot_id]);
  path_pub_[robot_id].publish(path_msg);
}

void KimeraCentralized::logRobotTrajectory(const size_t& robot_id,
                                           const std::string& filename) const {
  assert(robot_id < num_robots_);
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("KimeraCentralized: Error opening log file: " << filename);
    return;
  }

  file << "pose_index,qx,qy,qz,qw,tx,ty,tz\n";
  for (size_t i = 0; i < optimized_path_[robot_id].size(); i++) {
    const geometry_msgs::Pose& pose =
        GtsamPoseToRos(optimized_path_[robot_id][i]);
    file << i << ",";
    file << pose.orientation.x << ",";
    file << pose.orientation.y << ",";
    file << pose.orientation.z << ",";
    file << pose.orientation.w << ",";
    file << pose.position.x << ",";
    file << pose.position.y << ",";
    file << pose.position.z << "\n";
  }

  file.close();
}

}  // namespace kimera_distributed