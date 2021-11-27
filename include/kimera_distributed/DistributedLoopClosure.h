/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <kimera_distributed/utils.h>
#include <kimera_multi_lcd/LoopClosureDetector.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <iostream>
#include <map>
#include <memory>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <vector>

namespace lcd = kimera_multi_lcd;

namespace kimera_distributed {

class DistributedLoopClosure {
 public:
  DistributedLoopClosure(const ros::NodeHandle& n);
  ~DistributedLoopClosure();

  inline void getLoopClosures(std::vector<lcd::VLCEdge>* loop_closures) {
    *loop_closures = loop_closures_;
  }

  inline ros::Time getLastCallbackTime() const { return last_callback_time_; }

  inline size_t getRobotId() const { return my_id_; }

  // For debugging purpose
  void saveLoopClosuresToFile(const std::string filename);

 private:
  ros::NodeHandle nh_;
  size_t my_id_;
  size_t num_robots_;
  bool use_actionlib_;

  bool log_output_;
  std::string log_output_dir_;
  std::vector<size_t> received_bow_bytes_;
  std::vector<size_t> received_vlc_bytes_;

  // Loop closure detector
  std::shared_ptr<lcd::LoopClosureDetector> lcd_;
  lcd::LcdParams lcd_params_;

  // Loop closures
  std::vector<lcd::VLCEdge> loop_closures_;

  // Last msg time
  ros::Time last_callback_time_;

  // ROS subscriber
  std::vector<ros::Subscriber> bow_subscribers;

  // ROS publisher
  ros::Publisher loop_closure_publisher_;

  /**
   * Callback to process bag of word vectors received from robots
   */
  void bowCallback(const kimera_vio_ros::BowQueryConstPtr& msg);

  /*! \brief Request multiple full visual loop closure frames 
   * (including the 3d keypoints)
   *  - vertex_ids: ids of vertices of VLC frame in question
   */
  bool requestVLCFrame(const lcd::RobotPoseIdSet& vertex_ids);

  /**
   * @brief Request multiple VLC frames using ROS service
   * @param vertex_ids
   * @return
   */
  bool requestVLCFrameService(const lcd::RobotPoseIdSet& vertex_ids);

  /**
   * @brief Request multiple VLC frames using actionlib
   * @param vertex_ids
   * @return
   */
  bool requestVLCFrameAction(const lcd::RobotPoseIdSet& vertex_ids);

  // Publish detected loop closure
  void publishLoopClosure(const lcd::VLCEdge& loop_closure_edge);
  
  /**
   * Callback to request VLC frames in batch
   */
  void requestFramesCallback(const ros::TimerEvent &event);

  /**
   * Callback for goemetric verification of potential loop closures
   */
  void verifyLoopCallback(const ros::TimerEvent &event);

  /**
   * Log communication stats to file
   */ 
  void logCommStat(const std::string& filename);

  // List of potential loop closures 
  // that require to request VLC frames
  std::vector<lcd::PotentialVLCEdge> potential_lcs_;
  
  // List of potential loop closures 
  // that are ready for local geometric verification
  std::vector<lcd::PotentialVLCEdge> potential_lcs_ready_;

  // Maximum number of VLC frames to request in one batch
  int vlc_batch_size_;

  // Map from robot ID to name
  std::map<size_t, std::string> robot_names_;

  // Timer to request VLC frames
  double request_sleeptime_;
  ros::Timer request_timer_;

  // Timer to perform geometric verification
  double verify_sleeptime_;
  ros::Timer verify_timer_;
};

}  // namespace kimera_distributed