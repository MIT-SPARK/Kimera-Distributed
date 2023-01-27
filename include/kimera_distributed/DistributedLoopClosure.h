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

#include <pose_graph_tools/BowQueries.h>
#include <pose_graph_tools/BowRequests.h>
#include <pose_graph_tools/VLCFrames.h>
#include <pose_graph_tools/VLCRequests.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphQuery.h>
#include <pose_graph_tools/LoopClosures.h>
#include <pose_graph_tools/LoopClosuresAck.h>
#include <kimera_multi_lcd/LoopClosureDetector.h>
#include <kimera_distributed/utils.h>
#include <kimera_distributed/SubmapAtlas.h>
#include <std_msgs/UInt16MultiArray.h>

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
  // Yulun: commented out the mutex as only one thread is accessing submap_atlas_
  // std::mutex submap_atlas_mutex_;

  // Bag of words vectors
  std::unordered_map<lcd::RobotId, lcd::PoseId> bow_latest_; // Latest BoW received from each robot
  std::unordered_map<lcd::RobotId, std::unordered_set<lcd::PoseId>> bow_received_; 
  std::vector<pose_graph_tools::BowQuery> bow_msgs_; // New BoW messages that need to be processed
  std::mutex bow_msgs_mutex_;

  // Loop closure detector
  std::shared_ptr<lcd::LoopClosureDetector> lcd_;
  lcd::LcdParams lcd_params_;
  std::mutex lcd_mutex_;

  // Loop closures
  int num_inter_robot_loops_;
  gtsam::NonlinearFactorGraph keyframe_loop_closures_;
  gtsam::NonlinearFactorGraph submap_loop_closures_;
  // New loop closures to be synchronized with other robots
  std::map<lcd::EdgeID, gtsam::BetweenFactor<gtsam::Pose3>, lcd::CompareEdgeID> submap_loop_closures_queue_;
  // Edge IDs of all established loop closures
  std::set<lcd::EdgeID, lcd::CompareEdgeID> submap_loop_closures_ids_;

  // List of potential loop closures
  // that require to request VLC frames
  std::mutex vlc_service_mutex_;
  std::unordered_map<size_t, std::vector<lcd::PotentialVLCEdge> > candidate_lc_;
  std::mutex candidate_lc_mutex_;
  std::queue<lcd::PotentialVLCEdge> queued_lc_;

  // Keep track of BoW vectors requested by other robots
  std::map<lcd::RobotId, std::set<lcd::PoseId>> requested_bows_;
  std::mutex requested_bows_mutex_;

  // Keep track of VLC frames requested by other robots
  std::map<lcd::RobotId, std::set<lcd::PoseId>> requested_frames_;
  std::mutex requested_frames_mutex_;

  // Parameters controlling communication due to VLC request/response
  int bow_batch_size_;  // Maximum number of Bow vectors per robot to request in one batch
  int vlc_batch_size_;  // Maximum number of VLC frames per robot to request in one batch
  int loop_batch_size_; // Maximum number of loop closures to synchronize in one batch
  int comm_sleep_time_;  // Sleep time of communication thread
  int loop_sync_sleep_time_; // Sleep time of loop synchronization
  int detection_batch_size_;  // Maximum number of loop detection to perform in one batch
  int bow_skip_num_;  // Request every bow_skip_num_ bow vectors

  // Map from robot ID to name
  std::map<size_t, std::string> robot_names_;

  // Record if robot is currently connected
  std::map<size_t, bool> robot_connected_;

  // Record if loop closure publisher is initialized
  std::map<size_t, bool> loop_pub_initialized_;

  // Number of updates received from back-end
  int backend_update_count_;

  // ROS subscriber
  ros::Subscriber local_pg_sub_;
  ros::Subscriber internal_vlc_sub_;
  std::vector<ros::Subscriber> bow_sub_;
  std::vector<ros::Subscriber> bow_requests_sub_;
  std::vector<ros::Subscriber> vlc_requests_sub_;
  std::vector<ros::Subscriber> vlc_responses_sub_;
  std::vector<ros::Subscriber> loop_sub_;
  std::vector<ros::Subscriber> loop_ack_sub_;
  ros::Subscriber dpgo_sub_;
  ros::Subscriber connectivity_sub_;

  // ROS publisher
  ros::Publisher vlc_responses_pub_;
  ros::Publisher vlc_requests_pub_;
  ros::Publisher pose_graph_pub_;
  ros::Publisher bow_requests_pub_;
  ros::Publisher bow_response_pub_;
  ros::Publisher loop_pub_;
  ros::Publisher loop_ack_pub_;

  // For incremental publishing
  size_t last_get_submap_idx_;
  size_t last_get_lc_idx_;

  // ROS service
  ros::ServiceServer pose_graph_request_server_;

  // Timer
  ros::Timer log_timer_;
  ros::Time start_time_;
  ros::Time next_loop_sync_time_;
  ros::Time next_latest_bow_pub_time_;

  // Threads
  std::unique_ptr<std::thread> detection_thread_;
  std::unique_ptr<std::thread> verification_thread_;
  std::unique_ptr<std::thread> comms_thread_;

  // Logging
  std::ofstream odometry_file_;           // log received odometry poses from VIO
  std::ofstream loop_closure_file_;       // log inter-robot loop closures
  std::ofstream lcd_log_file_;            // log loop closure statistics
  size_t bow_backlog_, vlc_backlog_;      // Current backlog (number of missing BoW and VLC frames)
  std::vector<size_t> num_loops_with_robot_;
 private:
  /**
   * @brief Run place recognition / loop detection spin
   */
  void runDetection();

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
  void bowCallback(const pose_graph_tools::BowQueriesConstPtr& query_msg);

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
   * Callback to process internal VLC frames
   */
  void internalVLCCallback(const pose_graph_tools::VLCFramesConstPtr& msg);

  /**
   * @brief Callback to process the BoW requests from other robots
   * @param msg
   */
  void bowRequestsCallback(const pose_graph_tools::BowRequestsConstPtr& msg);

  /**
   * Callback to process the VLC requests from other robots
   */
  void vlcRequestsCallback(const pose_graph_tools::VLCRequestsConstPtr& msg);

  /**
   * Callback to process new inter-robot loop closures
  */
  void loopClosureCallback(const pose_graph_tools::LoopClosuresConstPtr &msg);

  /**
   * Callback to process new inter-robot loop closures
  */
  void loopAcknowledgementCallback(const pose_graph_tools::LoopClosuresAckConstPtr &msg);

  /**
   * @brief Callback to receive optimized submap poses from dpgo
   * @param msg
   */
  void dpgoCallback(const nav_msgs::PathConstPtr& msg);

  /**
   * @brief Callback to timer used for periodically logging
  */
  void logTimerCallback(const ros::TimerEvent &event);

  /**
   * @brief Subscriber callback that listens to the list of currently connected robots
  */
  void connectivityCallback(const std_msgs::UInt16MultiArrayConstPtr &msg);

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
   * Initialize loop closures
   */
  void initializeLoopPublishers();

  /**
   * Publish queued loop closures to be synchronized with other robots
   */
  void publishQueuedLoops();

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
   * @brief Request BoW vectors
   */
  void requestBowVectors();

  /**
   * @brief Publish BoW vectors requested by other robots
   */
  void publishBowVectors();

  /**
   * @brief Publish the latest BoW vector of this robot
  */
  void publishLatestBowVector();

  /**
   * Check and submit VLC requests
   */
  void requestFrames();

  /**
   * @brief Publish VLC frames requested by other robots
   */
  void publishFrames();

  /**
   * @brief Detect loop closure using BoW vectors
   */
  void detectLoopCallback();

  /**
   * @brief Detect loop closure using the input BoW vector
   * @param vertex_query the unique ID associated with the frame of this BoW vector
   * @param bow_vec query bag of words vector
   */
  void detectLoop(const lcd::RobotPoseId& vertex_query, const DBoW2::BowVector& bow_vec);

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
   * @brief Create log files
   */
  void createLogFiles();
  /**
   * @brief Close all log files
   */
  void closeLogFiles();
  /**
   * @brief Log latest LCD statistics
  */
  void logLcdStat();
  /**
   * @brief Log a keyframe pose in odometry frame
   * @param symbol_frame
   * @param T_odom_frame
   */
  void logOdometryPose(const gtsam::Symbol &symbol_frame, const gtsam::Pose3 &T_odom_frame);
  /**
   * @brief Log a loop closure to file
   * @param symbol_src
   * @param symbol_dst
   * @param T_src_dst
   */
  void logLoopClosure(const gtsam::Symbol &symbol_src, const gtsam::Symbol &symbol_dst, const gtsam::Pose3 &T_src_dst);
  /**
   * @brief Save the latest pose estimates to the world frame
   * @brief filename Output file
  */
  void savePosesInWorldFrame(const std::string &filename) const;
  /**
   * @brief Save the current submap atlas to file.
   * This function saves the poses of all keyframes in their respective submaps
   * @param filename Output file
  */
  void saveSubmapAtlas(const std::string &filename) const;
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

  /**
   * Randomly sleep from (min_sec, max_sec) seconds
   */
  void randomSleep(double min_sec, double max_sec);
};

}  // namespace kimera_distributed