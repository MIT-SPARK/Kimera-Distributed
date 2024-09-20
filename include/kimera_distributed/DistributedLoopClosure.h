/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu) Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <kimera_multi_lcd/loop_closure_detector.h>
#include <pose_graph_tools_msgs/BowQueries.h>
#include <pose_graph_tools_msgs/BowRequests.h>
#include <pose_graph_tools_msgs/LoopClosures.h>
#include <pose_graph_tools_msgs/LoopClosuresAck.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_msgs/PoseGraphQuery.h>
#include <pose_graph_tools_msgs/VLCFrames.h>
#include <pose_graph_tools_msgs/VLCRequests.h>

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "kimera_distributed/SubmapAtlas.h"
#include "kimera_distributed/configs.h"
#include "kimera_distributed/utils.h"

namespace lcd = kimera_multi_lcd;

namespace kimera_distributed {

class DistributedLoopClosure {
 public:
  DistributedLoopClosure();
  ~DistributedLoopClosure();

  void initialize(const DistributedLoopClosureConfig& config);

 protected:
  DistributedLoopClosureConfig config_;
  std::atomic<bool> should_shutdown_{false};

  std::vector<size_t> received_bow_bytes_;
  std::vector<size_t> received_vlc_bytes_;

  // Submap Atlas
  std::unique_ptr<SubmapAtlas> submap_atlas_;
  // Yulun: commented out the mutex as only one thread is accessing submap_atlas_
  // std::mutex submap_atlas_mutex_;

  // Bag of words vectors
  std::unordered_map<lcd::RobotId, lcd::PoseId>
      bow_latest_;  // Latest BoW received from each robot
  std::unordered_map<lcd::RobotId, std::unordered_set<lcd::PoseId>> bow_received_;
  std::vector<pose_graph_tools_msgs::BowQuery>
      bow_msgs_;  // New BoW messages that need to be processed
  std::mutex bow_msgs_mutex_;

  // Loop closure detector
  std::shared_ptr<lcd::LoopClosureDetector> lcd_;
  std::mutex lcd_mutex_;

  // Loop closures
  int num_inter_robot_loops_;
  gtsam::NonlinearFactorGraph keyframe_loop_closures_;
  gtsam::NonlinearFactorGraph submap_loop_closures_;
  // New loop closures to be synchronized with other robots
  std::map<lcd::EdgeID, gtsam::BetweenFactor<gtsam::Pose3>, lcd::CompareEdgeID>
      submap_loop_closures_queue_;
  // Edge IDs of all established loop closures
  std::set<lcd::EdgeID, lcd::CompareEdgeID> submap_loop_closures_ids_;

  // Data structures for offline mode
  gtsam::NonlinearFactorGraph offline_keyframe_loop_closures_;
  std::map<lcd::RobotPoseId, pose_graph_tools_msgs::VLCFrameMsg> offline_robot_pose_msg_;

  // List of potential loop closures
  // that require to request VLC frames
  std::mutex vlc_service_mutex_;
  std::unordered_map<size_t, std::vector<lcd::PotentialVLCEdge>> candidate_lc_;
  std::mutex candidate_lc_mutex_;
  std::queue<lcd::PotentialVLCEdge> queued_lc_;

  // Keep track of BoW vectors requested by other robots
  std::map<lcd::RobotId, std::set<lcd::PoseId>> requested_bows_;
  std::mutex requested_bows_mutex_;

  // Keep track of VLC frames requested by other robots
  std::map<lcd::RobotId, std::set<lcd::PoseId>> requested_frames_;
  std::mutex requested_frames_mutex_;

  // Record if robot is currently connected
  std::map<size_t, bool> robot_connected_;

  // Record if loop closure publisher is initialized
  std::map<size_t, bool> loop_pub_initialized_;

  // Number of updates received from back-end
  int backend_update_count_;

  // Pose corrector
  gtsam::Pose3 T_world_dpgo_;

  // For incremental publishing
  size_t last_get_submap_idx_;
  size_t last_get_lc_idx_;

  // Threads
  std::unique_ptr<std::thread> detection_thread_;
  std::unique_ptr<std::thread> verification_thread_;
  std::unique_ptr<std::thread> comms_thread_;

  // Logging
  std::ofstream odometry_file_;      // log received odometry poses from VIO
  std::ofstream loop_closure_file_;  // log inter-robot loop closures
  std::ofstream lcd_log_file_;       // log loop closure statistics
  size_t bow_backlog_,
      vlc_backlog_;  // Current backlog (number of missing BoW and VLC frames)
  std::vector<size_t> num_loops_with_robot_;

 protected:
  /**
   * @brief Compute the transformation T_world_odom
   */
  gtsam::Pose3 getOdomInWorldFrame() const;
  /**
   * @brief Compute the transformation T_world_KF
   * KF denotes the coordinate of the latest keyframe in the submap atlas
   * This function is currently only used for debugging
   */
  gtsam::Pose3 getLatestKFInWorldFrame() const;
  /**
   * @brief Compute the transformation T_odom_KF
   * KF denotes the coordinate of the latest keyframe in the submap atlas
   * This function is currently only used for debugging
   */
  gtsam::Pose3 getLatestKFInOdomFrame() const;
  /**
   * Callback to process bag of word vectors received from robots
   */
  void processBow(const pose_graph_tools_msgs::BowQueriesConstPtr& query_msg);

  /**
   * @brief Subscribe to incremental pose graph of this robot published by VIO
   * @param msg
   */
  bool processLocalPoseGraph(const pose_graph_tools_msgs::PoseGraph::ConstPtr& msg);

  /**
   * Callback to process internal VLC frames
   */
  void processInternalVLC(const pose_graph_tools_msgs::VLCFramesConstPtr& msg);

  /**
   * @brief Callback to receive optimized submap poses from dpgo
   * @param msg
   */
  void processOptimizedPath(const nav_msgs::PathConstPtr& msg);

  /**
   * @brief Detect loop closure using BoW vectors
   */
  void detectLoopSpin();

  /**
   * @brief Detect loop closure using the input BoW vector
   * @param vertex_query the unique ID associated with the frame of this BoW vector
   * @param bow_vec query bag of words vector
   */
  void detectLoop(const lcd::RobotPoseId& vertex_query,
                  const DBoW2::BowVector& bow_vec);

  /**
   * Perform geometric verification
   */
  void verifyLoopSpin();

  /**
   * Get bow vectors to query
   */
  bool queryBowVectorsRequest(lcd::RobotId& selected_robot_id,
                              std::set<lcd::PoseId>& bow_vectors);

  /**
   * Get bow vectors to publish
   */
  bool queryBowVectorsPublish(lcd::RobotId& selected_robot_id,
                              lcd::RobotPoseIdSet& bow_vectors);

  /**
   * Get VLC frames to request
   */
  void queryFramesRequest(lcd::RobotPoseIdSet& local_vertex_ids,
                          lcd::RobotId& selected_robot_id,
                          lcd::RobotPoseIdSet& selected_vertex_ids) const;

  /**
   * Get VLC frames to publish
   */
  void queryFramesPublish(lcd::RobotId& selected_robot_id,
                          lcd::RobotPoseIdSet& selected_vertex_ids);

  /**
   * Get sparse pose graph
   */
  pose_graph_tools_msgs::PoseGraph getSubmapPoseGraph(bool incremental = false);

  /**
   * Update the candidate list and verification queue
   */
  size_t updateCandidateList();

  /**
   * Save the Bow vectors from this session
   */
  void saveBowVectors(const std::string& filepath) const;

  /**
   * Save the VLC frames from this session
   */
  void saveVLCFrames(const std::string& filepath) const;

  /**
   * Load Bow vectors into this session (! caution !)
   */
  void loadBowVectors(size_t robot_id, const std::string& bow_json);

  /**
   * Load VLC frames into this session (! caution !)
   */
  void loadVLCFrames(size_t robot_id, const std::string& vlc_json);

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
   * @param timestamp in ns
   */
  void logOdometryPose(const gtsam::Symbol& symbol_frame,
                       const gtsam::Pose3& T_odom_frame,
                       uint64_t ts);
  /**
   * @brief Log a loop closure to file
   * @param keyframe_edge The loop closure between two keyframes
   */
  void logLoopClosure(const lcd::VLCEdge& keyframe_edge);
  /**
   * @brief Compute the latest pose estimates to the world frame
   * @brief nodes Pointer of Values type to populate
   */
  void computePosesInWorldFrame(gtsam::Values::shared_ptr nodes) const;
  /**
   * @brief Save the latest pose estimates to the world frame, sorted by keyframe ID
   * @brief filename Output file
   */
  void saveSortedPosesToFile(const std::string& filename, const gtsam::Values& nodes) const;
  /**
   * @brief Save the latest pose estimates to the world frame
   * @brief filename Output file
   */
  void savePosesToFile(const std::string& filename, const gtsam::Values& nodes) const;
  /**
   * @brief Save the current submap atlas to file.
   * This function saves two file:
   * 1) kimera_distributed_keyframes.csv: the poses of all keyframes in their respective
   * submaps 2) kimera_distributed_submaps.csv: the poses of all submaps in the robot's
   * odometry frame
   * @param directory Output directory
   */
  void saveSubmapAtlas(const std::string& directory) const;
  /**
   * @brief Load keyframe poses from a file
   * @param pose_file
   */
  void loadOdometryFromFile(const std::string& pose_file);
  /**
   * @brief Load loop closures (between keyframes from file)
   * @param lc_file
   */
  void loadLoopClosuresFromFile(const std::string& lc_file);
  /**
   * @brief Process loop closures loaded from file
   */
  void processOfflineLoopClosures();
};

}  // namespace kimera_distributed
