/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <gtsam/slam/BetweenFactor.h>
#include <kimera_distributed/DistributedPcm.h>
#include <fstream>
#include <iostream>
#include <string>

namespace kimera_distributed {

DistributedPcm::DistributedPcm(const ros::NodeHandle& n) : nh_(n) {}

DistributedPcm::~DistributedPcm() {}

bool DistributedPcm::initialize(const RobotID& id, const uint32_t& num_robots) {
  my_id_ = id;
  num_robots_ = num_robots;
  assert(my_id_ >= 0);
  assert(num_robots_ > 0);

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
    ros::Subscriber sub = nh_.subscribe(
        topic, 10, &DistributedLoopClosure::odometryEdgeCallback, this);
    odom_edge_subscribers_.push_back(sub);
  }
}

void DistributedPcm::addLoop(const VLCEdge& loop_closure_edge) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  new_factors.add(VLCEdgeToGtsam(loop_closure_edge));

  pgo_->update(new_factors);
  return;
}

std::vector<VLCEdge> DistributedPcm::getInlierLoopclosures() const {
  // Extract loop closures from the last filtered pose graph
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
    gtsam::Symbol key(robot_id_to_prefix(robot_id), frame_id);

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
    gtsam::Symbol from_key(robot_id_to_prefix(robot_from), prev_node);
    gtsam::Symbol to_key(robot_id_to_prefix(robot_to), current_node);

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
}

void DistributedPcm::saveLoopClosuresToFile(const std::string& filename) {}

}  // namespace kimera_distributed