/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */
#define SLOW_BUT_CORRECT_BETWEENFACTOR
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
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
  Vector6 odom_precisions, lc_precisions;
  odom_precisions.head<3>().setConstant(odom_rot_precision);
  odom_precisions.tail<3>().setConstant(odom_trans_precision);
  odom_noise_ = gtsam::noiseModel::Diagonal::Precisions(odom_precisions);
  lc_precisions.head<3>().setConstant(lc_rot_precision);
  lc_precisions.tail<3>().setConstant(lc_trans_precision);
  loop_closure_noise_ = gtsam::noiseModel::Diagonal::Precisions(lc_precisions);

  // Pcm parameters
  double pcm_trans_threshold, pcm_rot_threshold;
  if (!ros::param::get("~pcm_threshold_translation", pcm_trans_threshold) ||
      !ros::param::get("~pcm_threshold_rotation", pcm_rot_threshold)) {
    ROS_ERROR("Failed to get required PCM parameters! Shutting down... ");
    ros::shutdown();
  }

  // Initialize pcm
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(pcm_trans_threshold, pcm_rot_threshold);
  pgo_params.logOutput(log_output_path_);
  pgo_params.setIncremental();
  pgo_ = std::unique_ptr<KimeraRPGO::RobustSolver>(
      new KimeraRPGO::RobustSolver(pgo_params));

  // Initialize publishers
  pose_graph_pub_ = nh_.advertise<pose_graph_tools::PoseGraph>(
      "pose_graph_optimized", 1, false);

  for (size_t i = 0; i < num_robots_; i++) {
    std::string path_topic = "robot_" + str(i) + "_optimized_path";
    path_pub_.push_back(nh_.advertise<nav_msgs::Path>(path_topic, 1, false));
  }
}

KimeraCentralized::~KimeraCentralized() {}

void KimeraCentralized::timerCallback(const ros::TimerEvent&) {
  // Request pose grah

  // Add and update rpgo

  // Publish

  // Log
}

bool KimeraCentralized::getNewPoseGraph(
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
    // Pick out the new odometry factors

    // Pick out the new loop closures

    // Pick out new shared loop closures and check for duplicates with other
    // robots
  }
}

bool KimeraCentralized::requestRobotPoseGraph(
    const size_t& robot_id,
    gtsam::NonlinearFactorGraph* odom_factors,
    gtsam::NonlinearFactorGraph* private_lc_factors,
    gtsam::NonlinearFactorGraph* shared_lc_factors,
    gtsam::Values* values) const {
  assert(NULL != odom_factors);
  assert(NULL != private_lc_factors);
  assert(NULL != shared_lc_factors);
  assert(NULL != values);
  // Request pose graph
  pose graph pose_graph_tools::PoseGraphQuery query;
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
    const gtsam::Pose3& meas = RosPoseToGtsam(edge.pose);
    const gtsam::Symbol from_key(robot_id_to_prefix(edge.robot_from),
                                 edge.key_from);
    const gtsam::Symbol to_key(robot_id_to_prefix(edge.robot_to), edge.key_to);

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

  ROS_INFO_STREAM(
      "Agent " << robot_id << " receives local pose graph with "
               << odom_factors->size() << " odometry edges and "
               << private_lc_factors->size() << " private loop closures and "
               << shared_lc_factors->size() << " shared loop closures.");

  return true;
}

void KimeraCentralized::publishPoseGraph() const {}

void KimeraCentralized::publishRobotTrajectory(const size_t& robot_id) const {}

bool KimeraCentralized::logRobotTrajectory(const size_t& robot_id,
                                           const std::string& filename) const {}

}  // namespace kimera_distributed