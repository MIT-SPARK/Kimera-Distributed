/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <gtsam/slam/BetweenFactor.h>
#include <kimera_distributed/DistributedPcm.h>
#include <kimera_distributed/prefix.h>
#include <kimera_distributed/types.h>

#include <fstream>
#include <iostream>
#include <string>

namespace kimera_distributed {

DistributedPcm::DistributedPcm(const ros::NodeHandle& n)
    : nh_(n), my_id_(-1), num_robots_(-1), b_is_frozen_(false) {
  int my_id_int = -1;
  int num_robots_int = -1;
  if (!ros::param::get("~robot_id", my_id_int) ||
      !ros::param::get("~num_robots", num_robots_int)) {
    ROS_ERROR("Distributed PCM failed to robot ID!");
    ros::shutdown();
  }
  assert(my_id_int >= 0);
  assert(num_robots_int > 0);
  my_id_ = my_id_int;
  num_robots_ = num_robots_int;

  b_request_from_robot_.assign(num_robots_, false);

  // Pcm parameters
  double pcm_trans_threshold, pcm_rot_threshold;
  if (!ros::param::get("~log_output_path", log_output_path_) ||
      !ros::param::get("~pcm_threshold_translation", pcm_trans_threshold) ||
      !ros::param::get("~pcm_threshold_rotation", pcm_rot_threshold)) {
    ROS_ERROR("PCM failed to get required parameters! Shutting down... ");
    ros::shutdown();
  }

  // Initialize pcm
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(pcm_trans_threshold, pcm_rot_threshold);
  pgo_params.logOutput(log_output_path_);
  pgo_ = std::unique_ptr<KimeraRPGO::RobustSolver>(
      new KimeraRPGO::RobustSolver(pgo_params));

  // Start odometry edge subscribers
  for (size_t id = 0; id < num_robots_; ++id) {
    std::string topic = "/kimera" + std::to_string(id) +
                        "/kimera_vio_ros/pose_graph_incremental";
    ros::Subscriber sub =
        nh_.subscribe(topic, 50, &DistributedPcm::odometryEdgeCallback, this);
    odom_edge_subscribers_.push_back(sub);
  }

  // Start loop closure edge subscribers
  std::string loop_closure_topic =
      "/kimera" + std::to_string(my_id_) + "/kimera_distributed/loop_closure";
  loop_closure_edge_subscriber_ = nh_.subscribe(
      loop_closure_topic, 30, &DistributedPcm::loopclosureCallback, this);

  // Initialize pose graph publisher
  pose_graph_pub_ =
      nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1, false);

  // Initialize serivce
  shared_lc_server_ = nh_.advertiseService(
      "shared_lc_query", &DistributedPcm::shareLoopClosuresCallback, this);
  pose_graph_request_server_ = nh_.advertiseService(
      "request_pose_graph", &DistributedPcm::requestPoseGraphCallback, this);

  ROS_INFO_STREAM("Distributed Kimera PCM node initialized (ID = "
                  << my_id_ << "). \n"
                  << "Parameters: \n"
                  << "pcm_threshold_translation = " << pcm_trans_threshold
                  << "\n"
                  << "pcm_threshold_rotation = " << pcm_rot_threshold);
}

DistributedPcm::~DistributedPcm() {}

void DistributedPcm::addLoopClosures(
    const std::vector<VLCEdge>& loop_closure_edges) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  for (auto edge : loop_closure_edges) {
    new_factors.add(VLCEdgeToGtsam(edge));
  }

  pgo_->update(new_factors, new_values, false);
  // TODO: Detect if the set of inliers changed

  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();

  return;
}

void DistributedPcm::addLoopClosure(const VLCEdge& loop_closure) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  new_factors.add(VLCEdgeToGtsam(loop_closure));

  pgo_->update(new_factors, new_values, false);
  pgo_->saveData(log_output_path_);
  // TODO: Detect if the set of inliers changed

  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();

  return;
}

std::vector<VLCEdge> DistributedPcm::getInlierLoopclosures(
    const gtsam::NonlinearFactorGraph& nfg) const {
  std::vector<VLCEdge> loop_closures;
  // Extract loop closures from the last filtered pose graph
  for (auto factor : nfg) {
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
            factor)) {
      // Check not odometry factor
      if (factor->front() + 1 != factor->back()) {
        gtsam::BetweenFactor<gtsam::Pose3> lc_edge =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
                factor);

        gtsam::Symbol src_key(lc_edge.front());
        gtsam::Symbol dst_key(lc_edge.back());
        VertexID src(robot_prefix_to_id.at(src_key.chr()), src_key.index());
        VertexID dst(robot_prefix_to_id.at(dst_key.chr()), dst_key.index());
        VLCEdge vlc_edge(src, dst, lc_edge.measured());
        loop_closures.push_back(vlc_edge);
      }
    }
  }

  return loop_closures;
}

void DistributedPcm::odometryEdgeCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  // New gtsam factors and values
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  // Iterate through nodes
  for (pose_graph_tools::PoseGraphNode pg_node : msg->nodes) {
    const gtsam::Pose3 estimate = RosPoseToGtsam(pg_node.pose);
    const uint32_t robot_id = pg_node.robot_id;
    const uint32_t frame_id = pg_node.key;
    gtsam::Symbol key(robot_id_to_prefix.at(robot_id), frame_id);

    if (!values_.exists(key)) new_values.insert(key, estimate);
  }

  // Iterate through edge
  for (pose_graph_tools::PoseGraphEdge pg_edge : msg->edges) {
    // Get edge information
    const gtsam::Pose3 measure = RosPoseToGtsam(pg_edge.pose);
    const uint32_t prev_node = pg_edge.key_from;
    const uint32_t current_node = pg_edge.key_to;
    const uint32_t robot_from = pg_edge.robot_from;
    const uint32_t robot_to = pg_edge.robot_to;
    gtsam::Symbol from_key(robot_id_to_prefix.at(robot_from), prev_node);
    gtsam::Symbol to_key(robot_id_to_prefix.at(robot_to), current_node);

    // Check if odometry edge
    if (pg_edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
      // odometry edge
      if (robot_from != robot_to || current_node != prev_node + 1) {
        ROS_ERROR(
            "Odometry edge should connect two consecutive nodes from the same "
            "robot. ");
        continue;
      }

      // Create hard coded covariance
      static const gtsam::SharedNoiseModel& noise =
          gtsam::noiseModel::Isotropic::Variance(6, 1e-2);

      // Add to pcm TODO: covariance hard coded for now
      new_factors.add(
          gtsam::BetweenFactor<gtsam::Pose3>(from_key, to_key, measure, noise));
    }
  }

  pgo_->update(new_factors, new_values, false);
  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();
}

void DistributedPcm::loopclosureCallback(
    const pose_graph_tools::PoseGraphEdge::ConstPtr& msg) {
  VLCEdge new_loop_closure;
  VLCEdgeFromMsg(*msg, &new_loop_closure);

  addLoopClosure(new_loop_closure);

  // For debugging
  std::vector<VLCEdge> loop_closures = getInlierLoopclosures(nfg_);
  saveLoopClosuresToFile(loop_closures,
                         log_output_path_ + "pcm_loop_closures.csv");
}

void DistributedPcm::saveLoopClosuresToFile(
    const std::vector<VLCEdge>& loop_closures, const std::string& filename) {
  ROS_INFO_STREAM("Saving pcm processed loop closures to " << filename);
  std::ofstream file;
  file.open(filename);

  // file format
  file << "robot1,pose1,robot2,pose2,qx,qy,qz,qw,tx,ty,tz\n";

  for (size_t i = 0; i < loop_closures.size(); ++i) {
    VLCEdge edge = loop_closures[i];
    file << edge.vertex_src_.first << ",";
    file << edge.vertex_src_.second << ",";
    file << edge.vertex_dst_.first << ",";
    file << edge.vertex_dst_.second << ",";
    gtsam::Pose3 pose = edge.T_src_dst_;
    gtsam::Quaternion quat = pose.rotation().toQuaternion();
    gtsam::Point3 point = pose.translation();
    file << quat.x() << ",";
    file << quat.y() << ",";
    file << quat.z() << ",";
    file << quat.w() << ",";
    file << point.x() << ",";
    file << point.y() << ",";
    file << point.z() << "\n";
  }

  file.close();
}

void DistributedPcm::publishPoseGraph() const {
  pose_graph_tools::PoseGraph pose_graph_msg = GtsamGraphToRos(nfg_, values_);
  pose_graph_pub_.publish(pose_graph_msg);
}

void DistributedPcm::querySharedLoopClosures(
    std::vector<pose_graph_tools::PoseGraphEdge>* shared_lc) {
  for (size_t id = 0; id < my_id_; id++) {
    std::string service_name =
        "/kimera" + std::to_string(id) + "/distributed_pcm/shared_lc_query";
    requestSharedLoopClosures query;
    query.request.robot_id = my_id_;
    if (!ros::service::call(service_name, query)) {
      ROS_ERROR("Could not query shared loop closures. ");
    }
    *shared_lc = query.response.loop_closures;
  }
}

bool DistributedPcm::shareLoopClosuresCallback(
    kimera_distributed::requestSharedLoopClosures::Request& request,
    kimera_distributed::requestSharedLoopClosures::Response& response) {
  auto request_robot_id = request.robot_id;
  if (request_robot_id < my_id_) {
    ROS_ERROR_STREAM("Should not receive request from robot "
                     << request_robot_id);
    return false;
  }
  if (b_request_from_robot_[request_robot_id]) {
    ROS_ERROR("Distributed PCM received repeated loop closure requests!! ");
    return false;
  }
  b_request_from_robot_[request_robot_id] = true;
  // freeze set of loop closures
  if (!b_is_frozen_) {
    loop_closures_frozen_ = getInlierLoopclosures(nfg_);
    b_is_frozen_ = true;
  }

  for (auto lc_edge : loop_closures_frozen_) {
    if (lc_edge.vertex_src_.first == request_robot_id ||
        lc_edge.vertex_dst_.first == request_robot_id) {
      pose_graph_tools::PoseGraphEdge shared_lc_edge;
      VLCEdgeToMsg(lc_edge, &shared_lc_edge);
      response.loop_closures.push_back(shared_lc_edge);
    }
  }

  // Check if need to unfreeze loop closures
  unlockLoopClosuresIfNeeded();
  return true;
}

bool DistributedPcm::requestPoseGraphCallback(
    pose_graph_tools::PoseGraphQuery::Request& request,
    pose_graph_tools::PoseGraphQuery::Response& response) {
  // Check that id is from robot
  assert(request.robot_id == my_id_);
  if (b_request_from_robot_[request.robot_id]) {
    ROS_ERROR("Distributed PCM received repeated pose graph requests!");
    return false;
  }
  b_request_from_robot_[request.robot_id] = true;
  // freeze set of loop closures
  if (!b_is_frozen_) {
    loop_closures_frozen_ = getInlierLoopclosures(nfg_);
    b_is_frozen_ = true;
  }

  std::vector<pose_graph_tools::PoseGraphEdge> interrobot_lc;
  querySharedLoopClosures(&interrobot_lc);

  const pose_graph_tools::PoseGraph pose_graph_msg =
      GtsamGraphToRos(nfg_, values_);

  // Filter out odometry set not from this robot
  // and add interrobot loop closures
  pose_graph_tools::PoseGraph out_graph;
  out_graph.nodes = pose_graph_msg.nodes;

  for (auto e : pose_graph_msg.edges) {
    if (e.type == pose_graph_tools::PoseGraphEdge::ODOM &&
        e.robot_from != my_id_) {
      // pass odometry edges without requested robot id
    } else {
      out_graph.edges.push_back(e);
    }
  }

  // For debugging
  std::vector<VLCEdge> output_loopclosures = loop_closures_frozen_;

  // Push the interrobot loop closures
  for (auto e : interrobot_lc) {
    out_graph.edges.push_back(e);
    VLCEdge edge;
    VLCEdgeFromMsg(e, &edge);
    output_loopclosures.push_back(edge);
  }

  response.pose_graph = out_graph;
  // Save all loop closures to file
  saveLoopClosuresToFile(
      output_loopclosures,
      "/home/yunchang/catkin_ws/src/Kimera-Distributed/pcm_loop_closures_" +
          std::to_string(my_id_) + ".csv");

  unlockLoopClosuresIfNeeded();

  return true;
}

void DistributedPcm::unlockLoopClosuresIfNeeded() {
  bool should_unlock = true;
  for (size_t id = my_id_; id < b_request_from_robot_.size(); ++id) {
    if (!b_request_from_robot_[id]) {
      should_unlock = false;
      break;
    }
  }

  if (should_unlock) {
    b_is_frozen_ = false;
    b_request_from_robot_.assign(b_request_from_robot_.size(), false);
  }
}

}  // namespace kimera_distributed