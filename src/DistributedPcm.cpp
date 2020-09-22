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
    : nh_(n), my_id_(-1), num_robots_(-1) {
  int my_id_int = -1;
  int num_robots_int = -1;
  assert(ros::param::get("~robot_id", my_id_int));
  assert(ros::param::get("~num_robots", num_robots_int));
  assert(my_id_int >= 0);
  assert(num_robots_int > 0);
  my_id_ = my_id_int;
  num_robots_ = num_robots_int;

  // Pcm parameters
  double pcm_trans_threshold, pcm_rot_threshold;
  assert(ros::param::get("~pcm_threshold_translation", pcm_trans_threshold));
  assert(ros::param::get("~pcm_threshold_rotation", pcm_rot_threshold));

  // Initialize pcm
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(pcm_trans_threshold, pcm_rot_threshold);
  pgo_ = std::unique_ptr<KimeraRPGO::RobustSolver>(
      new KimeraRPGO::RobustSolver(pgo_params));

  // Start odometry edge subscribers
  for (size_t id = my_id_; id < num_robots_; ++id) {
    std::string topic = "/kimera" + std::to_string(id) +
                        "/kimera_vio_ros/pose_graph_incremental";
    ros::Subscriber sub =
        nh_.subscribe(topic, 10, &DistributedPcm::odometryEdgeCallback, this);
    odom_edge_subscribers_.push_back(sub);
  }

  pose_graph_pub_ =
      nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1, false);
}

DistributedPcm::~DistributedPcm() {}

void DistributedPcm::addLoopClosures(
    const std::vector<VLCEdge>& loop_closure_edges) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  for (auto edge : loop_closure_edges) {
    new_factors.add(VLCEdgeToGtsam(edge));
  }

  pgo_->update(new_factors);
  // TODO: Add option to not optimize
  // TODO: Detect if the set of inliers changed

  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();

  return;
}

std::vector<VLCEdge> DistributedPcm::getInlierLoopclosures() const {
  std::vector<VLCEdge> loop_closures;
  // Extract loop closures from the last filtered pose graph
  for (auto factor : nfg_) {
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

  pgo_->update(new_factors, new_values);
  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();
  
}

void DistributedPcm::saveLoopClosuresToFile(const std::string& filename) {
  std::vector<VLCEdge> loop_closures = getInlierLoopclosures();
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

}  // namespace kimera_distributed