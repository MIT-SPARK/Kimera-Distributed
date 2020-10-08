/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <kimera_distributed/types.h>
#include <kimera_distributed/utils.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <iostream>
#include <map>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <vector>

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

  // Database of BOW vectors from my trajectory
  std::unique_ptr<OrbDatabase> db_BoW_;
  uint32_t next_pose_id_;
  DBoW2::BowVector latest_bowvec_;

  // Database of BOW vectors from other robots
  std::unique_ptr<OrbDatabase> shared_db_BoW_;
  std::map<uint32_t, VertexID> shared_db_to_vertex_;

  // ORB extraction and matching members
  cv::Ptr<cv::DescriptorMatcher> orb_feature_matcher_;

  // Dictionary of VLC frames
  VLCFrameDict vlc_frames_;

  // List of discovered loop closures
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
  double geometric_verification_min_inlier_count_;
  double geometric_verification_min_inlier_percentage_;

  // Last msg time
  ros::Time last_callback_time_;

  // ROS subscriber
  std::vector<ros::Subscriber> bow_subscribers;

  // ROS publisher
  ros::Publisher loop_closure_publisher_;

  void bowCallback(const kimera_distributed::BowQueryConstPtr& msg);

  bool detectLoopInMyDB(const VertexID& vertex_query,
                        const DBoW2::BowVector bow_vector_query,
                        VertexID* vertex_match);

  bool detectLoopInSharedDB(const VertexID& vertex_query,
                            const DBoW2::BowVector bow_vector_query,
                            VertexID* vertex_match);

  void requestVLCFrame(const VertexID& vertex_id);

  void ComputeMatchedIndices(const VertexID& vertex_query,
                             const VertexID& vertex_match,
                             std::vector<unsigned int>* i_query,
                             std::vector<unsigned int>* i_match) const;

  bool recoverPose(const VertexID& vertex_query, const VertexID& vertex_match,
                   gtsam::Pose3* T_query_match);

  void publishLoopClosure(const VLCEdge& loop_closure_edge);
};

}  // namespace kimera_distributed