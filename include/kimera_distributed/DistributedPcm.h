/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <kimera_distributed/types.h>
#include <kimera_distributed/utils.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <vector>

#include <KimeraRPGO/RobustSolver.h>
#include <pose_graph_tools/PoseGraph.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace kimera_distributed {

class DistributedPcm {
 public:
  DistributedPcm(const ros::NodeHandle& n);
  ~DistributedPcm();

  bool initialize(const RobotID& id, const uint32_t& num_robots);

  void addLoop(const VLCEdge& loop_closure_edge);

  std::vector<VLCEdge> getInlierLoopclosures() const;

 private:
  ros::NodeHandle nh_;
  RobotID my_id_;
  uint32_t num_robots_;

  std::vector<ros::Subscriber> odom_edge_subscribers_;

  // Latest pcm processed pose graph
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;

  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;

  void odometryEdgeCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  // For debugging purpose
  void saveLoopClosuresToFile(const std::string& filename);
};

}  // namespace kimera_distributed