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
#include <string>
#include <vector>

#include <KimeraRPGO/RobustSolver.h>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace kimera_distributed {

class KimeraCentralized {
 public:
  KimeraCentralized(const ros::NodeHandle& n);
  ~KimeraCentralized();

 private:
  ros::NodeHandle nh_;
  int num_robots_;
  std::string log_output_path_;

  // Update timer
  ros::Timer update_timer_;

  // ROS publisher
  ros::Publisher pose_graph_pub_;
  std::vector<ros::Publisher> path_pub_;

  // Latest pcm processed pose graph
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;

  // Use fixed noise for lc and odom noise for now
  gtsam::SharedNoiseModel odom_noise_;
  gtsam::SharedNoiseModel loop_closure_noise_;

  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;

  void timerCallback(const ros::TimerEvent&);

  bool getNewPoseGraph(gtsam::NonlinearFactorGraph* new_factors,
                       gtsam::Values* new_values) const;

  bool requestRobotPoseGraph(const size_t& robot_id,
                             gtsam::NonlinearFactorGraph* odom_factors,
                             gtsam::NonlinearFactorGraph* private_lc_factors,
                             gtsam::NonlinearFactorGraph* shared_lc_factors,
                             gtsam::Values* values) const;

  void publishPoseGraph() const;

  void publishRobotTrajectory(const size_t& robot_id) const;

  bool logRobotTrajectory(const size_t& robot_id,
                          const std::string& filename) const;
};

}  // namespace kimera_distributed