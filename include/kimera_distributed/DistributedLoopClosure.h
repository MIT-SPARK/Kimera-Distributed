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

  void getLoopClosures(std::vector<VLCEdge>* loop_closures);

  inline ros::Time getLastCallbackTime() const { return last_callback_time_; }

  inline RobotID getRobotId() const { return my_id_; }

  // For debugging purpose
  void saveLoopClosuresToFile(const std::string filename);

 private:
  ros::NodeHandle nh_;
  RobotID my_id_;
  uint32_t num_robots_;
  uint32_t next_pose_id_; // next pose id in my local trajectory

  // ORB extraction and matching members
  cv::Ptr<cv::DescriptorMatcher> orb_feature_matcher_;

  std::unique_ptr<OrbDatabase> db_BoW_;
  DBoW2::BowVector latest_bowvec_;
  VLCFrameDict vlc_frames_;
  std::vector<VLCEdge> loop_closures_;

  // Parameters for visual loop closure detection
  float alpha_;
  int dist_local_;
  int max_db_results_;
  float base_nss_factor_;
  float min_nss_factor_;

  // Parameters for geometric verification
  int max_ransac_iterations_;
  double lowe_ratio_;
  double ransac_threshold_;
  double ransac_inlier_threshold_stereo_;

  // Last msg time 
  ros::Time last_callback_time_;

  // ROS subscriber
  std::vector<ros::Subscriber> bow_subscribers;

  // ROS publisher 
  ros::Publisher loop_closure_publisher_;

  void bowCallback(const kimera_distributed::BowQueryConstPtr& msg);

  bool detectLoop(const VertexID& vertex_query, const DBoW2::BowVector bow_vector_query, VertexID* vertex_match);

  void requestVLCFrame(const VertexID& vertex_id);

  void ComputeMatchedIndices(const VertexID& vertex_query,
                             const VertexID& vertex_match,
                             std::vector<unsigned int>* i_query,
                             std::vector<unsigned int>* i_match) const;

  bool recoverPose(const VertexID& vertex_query, const VertexID& vertex_match, gtsam::Pose3* T_query_match);

  void publishLoopClosure(const VLCEdge& loop_closure_edge);
  
};

}  // namespace kimera_distributed