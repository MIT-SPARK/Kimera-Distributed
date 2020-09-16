/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <kimera_distributed/types.h>
#include <kimera_distributed/utils.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <DBoW2/DBoW2.h>

namespace kimera_distributed {

class DistributedLoopClosure {

 public:
  DistributedLoopClosure(const ros::NodeHandle& n);
  ~DistributedLoopClosure();

 private:
  ros::NodeHandle nh_;
  RobotID my_id_;
  uint32_t num_robots_;
  uint32_t next_pose_id_; // next pose id in my local trajectory

  std::unique_ptr<OrbDatabase> db_BoW_;
  DBoW2::BowVector latest_bowvec_;
  VLCFrameDict vlc_frames_;

  // Parameters for visual loop closure detection
  float alpha_;
  int dist_local_;
  int max_db_results_;
  float base_nss_factor_;

  std::vector<ros::Subscriber> bow_subscribers;
  
  void bowCallback(const kimera_distributed::BowQueryConstPtr& msg);

  void requestVLCFrame(const VertexID vertex_id);
  
};

}  // namespace kimera_distributed