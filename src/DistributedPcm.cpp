/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

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
    ros::Subscriber sub =
        nh_.subscribe(topic, 10, &DistributedLoopClosure::bowCallback, this);
    odom_edge_subscribers_.push_back(sub);
  }
}

void DistributedPcm::addLoop(const VLCEdge& loop_closure_edge) {}

std::vector<VLCEdge> DistributedPcm::getInlierLoopclosures() const {}

void DistributedPcm::odometryEdgeCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {}

void DistributedPcm::saveLoopClosuresToFile(const std::string& filename) {}

}  // namespace kimera_distributed