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
}

void DistributedPcm::addLoop(const VLCEdge& loop_closure_edge) {}

std::vector<VLCEdge> DistributedPcm::getInlierLoopclosures() const {}

void DistributedPcm::odometryEdgeCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {}

void DistributedPcm::saveLoopClosuresToFile(const std::string& filename) {}

}  // namespace kimera_distributed