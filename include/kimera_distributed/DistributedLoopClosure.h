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
  lcd::LoopClosureDetector lcd_;
  lcd::LcdParams lcd_params_;

  // Loop closures
  std::vector<lcd::VLCEdge> loop_closures_;

  // Last msg time
  ros::Time last_callback_time_;

  // ROS subscriber
  std::vector<ros::Subscriber> bow_subscribers;

  // ROS publisher
  ros::Publisher loop_closure_publisher_;

  void bowCallback(const kimera_vio_ros::BowQueryConstPtr& msg);

  bool detectLoopInMyDB(const lcd::RobotPoseId& vertex_query,
                        const DBoW2::BowVector bow_vector_query,
                        lcd::RobotPoseId* vertex_match);

  bool detectLoopInSharedDB(const lcd::RobotPoseId& vertex_query,
                            const DBoW2::BowVector bow_vector_query,
                            lcd::RobotPoseId* vertex_match);

  bool requestVLCFrame(const lcd::RobotPoseId& vertex_id);

  /**
   * @brief Request a VLC frame using ROS service
   * @param vertex_id
   * @return
   */
  bool requestVLCFrameService(const lcd::RobotPoseId& vertex_id);

  /**
   * @brief Request a VLC frame using actionlib
   * @param vertex_id
   * @return
   */
  bool requestVLCFrameAction(const lcd::RobotPoseId& vertex_id);

  void publishLoopClosure(const lcd::VLCEdge& loop_closure_edge);

  void logCommStat(const std::string& filename);
};

}  // namespace kimera_distributed