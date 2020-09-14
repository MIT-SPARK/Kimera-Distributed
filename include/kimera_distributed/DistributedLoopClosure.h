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
  uint32_t my_id_;
  uint32_t num_robots_;

  std::unique_ptr<OrbDatabase> db_BoW_;
  std::vector<ros::Subscriber> bow_subscribers;

  void bowCallback(const kimera_distributed::BowQueryConstPtr& msg);
  
};

}  // namespace kimera_distributed