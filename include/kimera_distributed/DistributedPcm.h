/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <kimera_distributed/requestSharedLoopClosures.h>
#include <kimera_distributed/SharedLoopClosureAction.h>
#include <kimera_distributed/types.h>
#include <kimera_distributed/utils.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/server/simple_action_server.h>
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
  bool use_actionlib_;
  bool b_is_frozen_;
  bool b_offline_mode_;
  bool b_multirobot_initialization_;
  std::string log_output_path_;
  std::string offline_data_path_;

  // ROS subscriber
  std::vector<ros::Subscriber> odom_edge_subscribers_;
  ros::Subscriber loop_closure_edge_subscriber_;

  // ROS publisher
  ros::Publisher pose_graph_pub_;

  // ROS service
  ros::ServiceServer shared_lc_server_;
  ros::ServiceServer pose_graph_request_server_;
  ros::ServiceServer initialization_server_;

  // Action server
  actionlib::SimpleActionServer<kimera_distributed::SharedLoopClosureAction> lc_action_server_;
  kimera_distributed::SharedLoopClosureFeedback action_feedback_;
  kimera_distributed::SharedLoopClosureResult action_result_;

  // Latest pcm processed pose graph
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;

  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;

  // Handle to log files
  std::ofstream pose_file_;        // log received poses
  std::ofstream odom_file_;        // log received odometry

  void odometryEdgeCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  void loopclosureCallback(
      const pose_graph_tools::PoseGraphEdge::ConstPtr& msg);

  void querySharedLoopClosuresService(size_t robot_id, std::vector<pose_graph_tools::PoseGraphEdge>* shared_lc) const;
  void querySharedLoopClosuresAction(size_t robot_id, std::vector<pose_graph_tools::PoseGraphEdge>* shared_lc) const;
  void querySharedLoopClosures(std::vector<pose_graph_tools::PoseGraphEdge>* shared_lc) const;

  void shareLoopClosureActionCallback(const kimera_distributed::SharedLoopClosureGoalConstPtr &goal);

  bool shareLoopClosureServiceCallback(
      kimera_distributed::requestSharedLoopClosures::Request& request,
      kimera_distributed::requestSharedLoopClosures::Response& response);

  bool requestPoseGraphCallback(
      pose_graph_tools::PoseGraphQuery::Request& request,
      pose_graph_tools::PoseGraphQuery::Response& response);

  bool requestInitializationCallback(
      pose_graph_tools::PoseGraphQuery::Request& request,
      pose_graph_tools::PoseGraphQuery::Response& response);

  std::vector<VLCEdge> loop_closures_frozen_;
  std::vector<bool> b_request_from_robot_;
  void unlockLoopClosuresIfNeeded();

  // Logging 
  void createLogs();
  void closeLogs();
  void saveNewPosesToLog(const pose_graph_tools::PoseGraphNode& node);
  void saveNewOdometryToLog(const pose_graph_tools::PoseGraphEdge& edge);

  // Offline data loading
  void initializeOffline();

  // Load a set of poses from file. 
  // Each row (except first header row) corresponds to a pose in the following format: 
  // robot_index,pose_index,qx,qy,qz,qw,tx,ty,tz
  gtsam::Values loadPosesOffline(const std::string& filename);

  // Load a set of measurements from file
  // Each row (except first header row) corresponds to a measurement in the following format: 
  // robot1,pose1,robot2,pose2,qx,qy,qz,qw,tx,ty,tz
  // Additional columns in the file will be ignored
  gtsam::NonlinearFactorGraph loadMeasurementsOffline(const std::string& filename, bool is_odometry = false);
};

}  // namespace kimera_distributed