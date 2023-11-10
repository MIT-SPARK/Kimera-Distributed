/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu) Yulun Tian (yulun@mit.edu)
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

#include "kimera_distributed/DistributedLoopClosure.h"
#include "kimera_distributed/utils.h"

#include <pose_graph_tools/BowQueries.h>
#include <pose_graph_tools/BowRequests.h>
#include <pose_graph_tools/LoopClosures.h>
#include <pose_graph_tools/LoopClosuresAck.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphQuery.h>
#include <pose_graph_tools/VLCFrames.h>
#include <pose_graph_tools/VLCRequests.h>
#include <std_msgs/UInt16MultiArray.h>

#include <tf/transform_broadcaster.h>

namespace lcd = kimera_multi_lcd;

namespace kimera_distributed {

class DistributedLoopClosureRos : DistributedLoopClosure {
 public:
  DistributedLoopClosureRos(const ros::NodeHandle& n);
  ~DistributedLoopClosureRos();

 private:
  ros::NodeHandle nh_;
  std::atomic<bool> should_shutdown_{false};

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
  ros::Subscriber dpgo_frame_corrector_sub_;

  // ROS publisher
  ros::Publisher vlc_responses_pub_;
  ros::Publisher vlc_requests_pub_;
  ros::Publisher pose_graph_pub_;
  ros::Publisher bow_requests_pub_;
  ros::Publisher bow_response_pub_;
  ros::Publisher loop_pub_;
  ros::Publisher loop_ack_pub_;
  ros::Publisher optimized_nodes_pub_;
  ros::Publisher optimized_path_pub_;
  ros::Publisher dpgo_frame_corrector_pub_;

  // ROS service
  ros::ServiceServer pose_graph_request_server_;

  // Timer
  ros::Timer log_timer_;
  ros::Timer tf_timer_;
  ros::Time start_time_;
  ros::Time next_loop_sync_time_;
  ros::Time next_latest_bow_pub_time_;

  // TF broadcaster from world to robot's odom frame
  tf::TransformBroadcaster tf_broadcaster_;

 private:
  std::string latest_kf_frame_id_;
  std::string odom_frame_id_;
  std::string world_frame_id_;
  

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
  void loopClosureCallback(const pose_graph_tools::LoopClosuresConstPtr& msg);

  /**
   * Callback to process new inter-robot loop closures
   */
  void loopAcknowledgementCallback(
      const pose_graph_tools::LoopClosuresAckConstPtr& msg);

  /**
   * @brief Callback to receive optimized submap poses from dpgo
   * @param msg
   */
  void dpgoCallback(const nav_msgs::PathConstPtr& msg);

  /**
   * @brief Publish optimized nodes
   * @param msg
   */
  void publishOptimizedNodesAndPath(const gtsam::Values& nodes);

  /**
   * @brief Subscribe to T_world_dpgo (default to identity)
   * @param msg
   */
  void dpgoFrameCorrectionCallback(const geometry_msgs::Pose::ConstPtr& msg);

  /**
   * @brief Callback to timer used for periodically logging
   */
  void logTimerCallback(const ros::TimerEvent& event);

  /**
   * @brief Publish world to dpgo frame based on first robot
   */
  void publishWorldToDpgoCorrection();

  /**
   * @brief Callback to timer used for periodically publishing TF
   */
  void tfTimerCallback(const ros::TimerEvent& event);

  /**
   * @brief Subscriber callback that listens to the list of currently connected robots
   */
  void connectivityCallback(const std_msgs::UInt16MultiArrayConstPtr& msg);

  /**
   * @brief Send submap-level pose graph for distributed optimization
   * @param request
   * @param response
   * @return
   */
  bool requestPoseGraphCallback(pose_graph_tools::PoseGraphQuery::Request& request,
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

  void publishSubmapOfflineInfo();

  /**
   * Randomly sleep from (min_sec, max_sec) seconds
   */
  void randomSleep(double min_sec, double max_sec);

  /**
   * @brief Publish TF between world and odom
   */
  void publishOdomToWorld();
  
  /**
   * @brief Publish TF between world and latest keyframe
   */
  void publishLatestKFToWorld();

  /**
   * @brief Publish TF between odom and latest keyframe
   */
  void publishLatestKFToOdom();

  /**
   * @brief Save VLC frames and BoW vectors
   */
  void save();
};

}  // namespace kimera_distributed