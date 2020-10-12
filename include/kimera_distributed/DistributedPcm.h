/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <kimera_distributed/requestSharedLoopClosures.h>
#include <kimera_distributed/types.h>
#include <kimera_distributed/utils.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <vector>

#include <KimeraRPGO/RobustSolver.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphQuery.h>

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

  void addLoopClosures(const std::vector<VLCEdge>& loop_closure_edges);

  void addLoopClosure(const VLCEdge& loop_closure);

  std::vector<VLCEdge> getInlierLoopclosures(
      const gtsam::NonlinearFactorGraph& nfg) const;

  void publishPoseGraph() const;

  // For debugging purpose
  void saveLoopClosuresToFile(const std::vector<VLCEdge>& loop_closures,
                              const std::string& filename);

 private:
  ros::NodeHandle nh_;
  RobotID my_id_;
  int num_robots_;
  std::string log_output_path_;

  // ROS subscriber
  std::vector<ros::Subscriber> odom_edge_subscribers_;
  ros::Subscriber loop_closure_edge_subscriber_;

  // ROS publisher
  ros::Publisher pose_graph_pub_;

  // ROS service
  ros::ServiceServer shared_lc_server_;
  ros::ServiceServer pose_graph_request_server_;

  // Latest pcm processed pose graph
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;

  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;

  void odometryEdgeCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  void loopclosureCallback(
      const pose_graph_tools::PoseGraphEdge::ConstPtr& msg);

  void querySharedLoopClosures(
      std::vector<pose_graph_tools::PoseGraphEdge>* shared_lc);

  bool shareLoopClosuresCallback(
      kimera_distributed::requestSharedLoopClosures::Request& request,
      kimera_distributed::requestSharedLoopClosures::Response& response);

  bool requestPoseGraphCallback(
      pose_graph_tools::PoseGraphQuery::Request& request,
      pose_graph_tools::PoseGraphQuery::Response& response);

  std::vector<VLCEdge> loop_closures_frozen_;
  bool b_is_frozen_;
};

}  // namespace kimera_distributed