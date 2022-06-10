/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu) Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <ros/ros.h>
#include <ros/time.h>

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <pose_graph_tools/VLCFrames.h>
#include <pose_graph_tools/VLCRequests.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphQuery.h>
#include <kimera_multi_lcd/LoopClosureDetector.h>
#include <kimera_distributed/utils.h>
#include <kimera_distributed/SubmapAtlas.h>

namespace lcd = kimera_multi_lcd;

namespace kimera_distributed {

class DistributedLoopClosure {
 public:
  DistributedLoopClosure(const ros::NodeHandle& n);
  ~DistributedLoopClosure();
  inline size_t getRobotId() const { return my_id_; }

 private:
  ros::NodeHandle nh_;
  size_t my_id_;
  size_t num_robots_;
  std::string frame_id_;

  bool log_output_;
  std::string log_output_dir_;
  std::atomic<bool> should_shutdown_{false};
  bool run_offline_;  // true to load poses and loop closures from file

  std::vector<size_t> received_bow_bytes_;
  std::vector<size_t> received_vlc_bytes_;

  // Submap Atlas
  std::unique_ptr<SubmapAtlas> submap_atlas_;
  std::mutex submap_atlas_mutex_;

  // Loop closure detector
  std::shared_ptr<lcd::LoopClosureDetector> lcd_;
  lcd::LcdParams lcd_params_;
  std::mutex lcd_mutex_;

  // Loop closures
  int num_inter_robot_loops_;
  gtsam::NonlinearFactorGraph keyframe_loop_closures_;
  gtsam::NonlinearFactorGraph submap_loop_closures_;

  // List of potential loop closures
  // that require to request VLC frames
  std::mutex vlc_service_mutex_;
  std::unordered_map<size_t, std::vector<lcd::PotentialVLCEdge> > candidate_lc_;
  std::mutex candidate_lc_mutex_;
  std::queue<lcd::PotentialVLCEdge> queued_lc_;

  // List of VLC frame IDs requested by other robots
  std::set<size_t> requested_frames_;
  std::mutex requested_frames_mutex_;

  // Parameters controlling communication due to VLC request/response
  int vlc_batch_size_;  // Maximum number of VLC frames per robot to request in one batch
  int vlc_sleep_time_;  // Sleep time of communication thread

  // Map from robot ID to name
  std::map<size_t, std::string> robot_names_;

  // ROS subscriber
  ros::Subscriber local_pg_sub_;
  std::vector<ros::Subscriber> bow_sub_;
  std::vector<ros::Subscriber> vlc_requests_sub_;
  std::vector<ros::Subscriber> vlc_responses_sub_;
  ros::Subscriber dpgo_sub_;

  // ROS publisher
  ros::Publisher loop_closure_pub_;
  ros::Publisher vlc_responses_pub_;
  ros::Publisher vlc_requests_pub_;
  ros::Publisher pose_graph_pub_;

  // For incremental publishing
  size_t last_get_submap_idx_;
  size_t last_get_lc_idx_;

  // ROS service
  ros::ServiceServer pose_graph_request_server_;

  // Threads
  std::unique_ptr<std::thread> verification_thread_;
  std::unique_ptr<std::thread> comms_thread_;

  // Logging
  std::ofstream keyframe_pose_file_;        // log received keyframe poses
  std::ofstream loop_closure_file_;       // log inter-robot loop closures
 private:
  /**
   * Run Geometric verification spin
   */
  void runVerification();

  /**
   * Run Comms spin to send requests
   */
  void runComms();

  /**
   * Callback to process bag of word vectors received from robots
   */
  void bowCallback(const pose_graph_tools::BowQueryConstPtr& msg);

  /**
   * @brief Subscribe to incremental pose graph of this robot published by VIO
   * @param msg
   */
  void localPoseGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  /**
   * Callback to process the VLC responses to our requests
   */
  void vlcResponsesCallback(const pose_graph_tools::VLCFramesConstPtr& msg);

  /**
   * Callback to process the VLC requests from other robots
   */
  void vlcRequestsCallback(const pose_graph_tools::VLCRequestsConstPtr& msg);

  /**
   * @brief Callback to receive optimized submap poses from dpgo
   * @param msg
   */
  void dpgoCallback(const nav_msgs::PathConstPtr& msg);

  /**
   * @brief Send submap-level pose graph for distributed optimization
   * @param request
   * @param response
   * @return
   */
  bool requestPoseGraphCallback(
      pose_graph_tools::PoseGraphQuery::Request& request,
      pose_graph_tools::PoseGraphQuery::Response& response);

  /**
   * Publish detected loop closure
   */
  void publishLoopClosure(const lcd::VLCEdge& loop_closure_edge);

  /**
   * Publish VLC requests
   */
  void processVLCRequests(const size_t& robot_id,
                          const lcd::RobotPoseIdSet& vertex_ids);

  /**
   * Publish VLC Frame requests from other robots
   */
  void publishVLCRequests(const size_t& robot_id,
                          const lcd::RobotPoseIdSet& vertex_ids);

  /**
   * Request local VLC Frames from Kimera-VIO-ROS
   */
  bool requestVLCFrameService(const lcd::RobotPoseIdSet& vertex_ids);

  /**
   * Check and submit VLC requests
   */
  void requestFrames();

  /**
   * Perform geometric verification
   */
  void verifyLoopCallback();

  /**
   * Get sparse pose graph
   */
  pose_graph_tools::PoseGraph getSubmapPoseGraph(bool incremental = false);

  /**
   * Update the candidate list and verification queue
   */
  size_t updateCandidateList();

  /**
   * Log communication stats to file
   */
  void logCommStat(const std::string& filename);
  /**
   * @brief Create log files
   */
  void createLogFiles();
  /**
   * @brief Close all log files
   */
  void closeLogFiles();
  /**
   * @brief Log a keyframe to file
   * @param symbol_frame
   * @param T_odom_frame
   */
  void logKeyframePose(const gtsam::Symbol &symbol_frame, const gtsam::Pose3 &T_odom_frame);
  /**
   * @brief Log a loop closure to file
   * @param symbol_src
   * @param symbol_dst
   * @param T_src_dst
   */
  void logLoopClosure(const gtsam::Symbol &symbol_src, const gtsam::Symbol &symbol_dst, const gtsam::Pose3 &T_src_dst);
  /**
   * @brief Load keyframe poses from a file
   * @param pose_file
   */
  void loadKeyframeFromFile(const std::string &pose_file);
  /**
   * @brief Load loop closures (between keyframes from file)
   * @param lc_file
   */
  void loadLoopClosuresFromFile(const std::string &lc_file);
};

}  // namespace kimera_distributed