/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu) Yun Chang (yunchang@mit.edu)
 */

#include "kimera_distributed/DistributedLoopClosure.h"

#include <DBoW2/DBoW2.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <kimera_multi_lcd/io.h>
#include <kimera_multi_lcd/utils.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/VLCFrameQuery.h>
#include <pose_graph_tools/utils.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <string>

namespace kimera_distributed {

DistributedLoopClosure::DistributedLoopClosure()
    : lcd_(new lcd::LoopClosureDetector),
      num_inter_robot_loops_(0),
      backend_update_count_(0),
      last_get_submap_idx_(0),
      last_get_lc_idx_(0),
      bow_backlog_(0),
      vlc_backlog_(0) {}

DistributedLoopClosure::~DistributedLoopClosure() {
  should_shutdown_ = true;

  if (detection_thread_) {
    detection_thread_->join();
    detection_thread_.reset();
  }

  if (verification_thread_) {
    verification_thread_->join();
    verification_thread_.reset();
  }

  if (comms_thread_) {
    comms_thread_->join();
    comms_thread_.reset();
  }
}

void DistributedLoopClosure::initialize(const DistributedLoopClosureConfig& config) {
  config_ = config;
  num_loops_with_robot_.assign(config_.num_robots_, 0);

  // Used for logging
  received_bow_bytes_.clear();
  received_vlc_bytes_.clear();

  // Load robot names and initialize candidate lc queues
  for (size_t id = 0; id < config_.num_robots_; id++) {
    candidate_lc_[id] = std::vector<lcd::PotentialVLCEdge>{};
    loop_pub_initialized_[id] = false;
  }

  // Initialize LCD
  lcd_->loadAndInitialize(config_.lcd_params_);

  // Initialize submap atlas
  submap_atlas_.reset(new SubmapAtlas(config_.submap_params_));

  if (config_.run_offline_) {
    for (size_t id = config.my_id_; id < config.num_robots_; ++id) {
      loadBowVectors(
          id,
          config.offline_dir_ + "robot_" + std::to_string(id) + "_bow_vectors.json");
      loadVLCFrames(
          id, config.offline_dir_ + "robot_" + std::to_string(id) + "_vlc_frames.json");
    }

    // Load odometry
    loadOdometryFromFile(config_.offline_dir_ + "odometry_poses.csv");
    // Load original loop closures between keyframes
    loadLoopClosuresFromFile(config_.offline_dir_ + "loop_closures.csv");
  } else {
    // Initialize log files to record keyframe poses and loop closures.
    if (config_.log_output_) {
      createLogFiles();
    }
  }

  // Initially assume all robots are connected
  for (size_t robot_id = 0; robot_id < config_.num_robots_; ++robot_id) {
    robot_connected_[robot_id] = true;
  }
}

void DistributedLoopClosure::processBow(
    const pose_graph_tools::BowQueriesConstPtr& query_msg) {
  for (const auto& msg : query_msg->queries) {
    lcd::RobotId robot_id = msg.robot_id;
    lcd::PoseId pose_id = msg.pose_id;
    // This robot is responsible for detecting loop closures with others with a larger
    // ID
    CHECK_GE(robot_id, config_.my_id_);
    lcd::RobotPoseId vertex_query(robot_id, pose_id);
    if (bow_received_[robot_id].find(pose_id) != bow_received_[robot_id].end()) {
      // Skip if this vector has been received before
      continue;
    }
    bow_latest_[robot_id] = std::max(bow_latest_[robot_id], pose_id);
    bow_received_[robot_id].emplace(pose_id);
    {  // start of BoW critical section
      std::unique_lock<std::mutex> bow_lock(bow_msgs_mutex_);
      bow_msgs_.push_back(msg);
    }  // end BoW critical section
  }
}

bool DistributedLoopClosure::processLocalPoseGraph(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  // Parse odometry edges and create new keyframes in the submap atlas
  std::vector<pose_graph_tools::PoseGraphEdge> local_loop_closures;
  const uint64_t ts = msg->header.stamp.toNSec();

  bool incremental_pub = true;
  if (submap_atlas_->numSubmaps() == 0 && !msg->nodes.empty()) {
    // Create the first keyframe
    // Start submap critical section
    // std::unique_lock<std::mutex> submap_lock(submap_atlas_mutex_);

    gtsam::Rot3 init_rotation(msg->nodes[0].pose.orientation.w,
                              msg->nodes[0].pose.orientation.x,
                              msg->nodes[0].pose.orientation.y,
                              msg->nodes[0].pose.orientation.z);
    gtsam::Point3 init_position(msg->nodes[0].pose.position.x,
                                msg->nodes[0].pose.position.y,
                                msg->nodes[0].pose.position.z);
    gtsam::Pose3 init_pose(init_rotation, init_position);

    if (config_.my_id_ == 0) {
      // Implicit assumption: first pose of the first robot in DPGO is set to identity
      T_world_dpgo_ = init_pose;
    }

    submap_atlas_->createKeyframe(0, init_pose, ts);
    logOdometryPose(
        gtsam::Symbol(robot_id_to_prefix.at(config_.my_id_), 0), init_pose, ts);
    incremental_pub = false;
  }

  // Extract timestamps of each new pose node
  std::map<int, uint64_t> node_timestamps;
  for (const auto& pg_node : msg->nodes) {
    node_timestamps[(int)pg_node.key] = pg_node.header.stamp.toNSec();
  }

  for (const auto& pg_edge : msg->edges) {
    if (pg_edge.robot_from == config_.my_id_ && pg_edge.robot_to == config_.my_id_ &&
        pg_edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
      int frame_src = (int)pg_edge.key_from;
      int frame_dst = (int)pg_edge.key_to;
      CHECK_EQ(frame_src + 1, frame_dst);
      if (submap_atlas_->hasKeyframe(frame_src) &&
          !submap_atlas_->hasKeyframe(frame_dst)) {
        // Start submap critical section
        // std::unique_lock<std::mutex> submap_lock(submap_atlas_mutex_);
        // Check that the next keyframe has the expected id
        int expected_frame_id = submap_atlas_->numKeyframes();
        if (frame_dst != expected_frame_id) {
          LOG(ERROR) << "Received unexpected keyframe! (received " << frame_dst
                     << ", expected " << expected_frame_id << ")";
        }

        // Use odometry to initialize the next keyframe
        lcd::VLCEdge keyframe_odometry;
        kimera_multi_lcd::VLCEdgeFromMsg(pg_edge, &keyframe_odometry);
        const auto T_src_dst = keyframe_odometry.T_src_dst_;
        const auto T_odom_src =
            submap_atlas_->getKeyframe(frame_src)->getPoseInOdomFrame();
        const auto T_odom_dst = T_odom_src * T_src_dst;
        uint64_t node_ts = ts;
        if (node_timestamps.find(frame_dst) != node_timestamps.end())
          node_ts = node_timestamps[frame_dst];
        submap_atlas_->createKeyframe(frame_dst, T_odom_dst, node_ts);

        // Save keyframe pose to file
        gtsam::Symbol symbol_dst(robot_id_to_prefix.at(config_.my_id_), frame_dst);
        logOdometryPose(symbol_dst, T_odom_dst, node_ts);
      }
    } else if (pg_edge.robot_from == config_.my_id_ &&
               pg_edge.robot_to == config_.my_id_ &&
               pg_edge.type == pose_graph_tools::PoseGraphEdge::LOOPCLOSE) {
      local_loop_closures.push_back(pg_edge);
    }
  }

  // Parse intra-robot loop closures
  for (const auto& pg_edge : local_loop_closures) {
    // Read loop closure between the keyframes
    lcd::VLCEdge keyframe_loop_closure;
    kimera_multi_lcd::VLCEdgeFromMsg(pg_edge, &keyframe_loop_closure);
    logLoopClosure(keyframe_loop_closure);
    const auto T_f1_f2 = keyframe_loop_closure.T_src_dst_;
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(6, 1e-2);

    {
      // Start submap critical section
      // std::unique_lock<std::mutex> submap_lock(submap_atlas_mutex_);
      // Convert the loop closure to between the corresponding submaps
      const auto keyframe_src = submap_atlas_->getKeyframe(pg_edge.key_from);
      const auto keyframe_dst = submap_atlas_->getKeyframe(pg_edge.key_to);
      if (!keyframe_src || !keyframe_dst) {
        LOG(ERROR) << "Received intra loop closure but keyframe does not exist!";
        continue;
      }
      const auto submap_src = CHECK_NOTNULL(keyframe_src->getSubmap());
      const auto submap_dst = CHECK_NOTNULL(keyframe_dst->getSubmap());
      gtsam::Symbol from_key(robot_id_to_prefix.at(config_.my_id_), submap_src->id());
      gtsam::Symbol to_key(robot_id_to_prefix.at(config_.my_id_), submap_dst->id());
      // Skip this loop closure if two submaps are identical or consecutive
      if (std::abs(submap_src->id() - submap_dst->id()) <= 1) continue;
      // Skip this loop closure if a loop closure already exists between the two submaps
      if (hasBetweenFactor(submap_loop_closures_, from_key, to_key)) continue;
      const auto T_s1_f1 = keyframe_src->getPoseInSubmapFrame();
      const auto T_s2_f2 = keyframe_dst->getPoseInSubmapFrame();
      const auto T_s1_s2 = T_s1_f1 * T_f1_f2 * (T_s2_f2.inverse());
      // Convert the loop closure to between the corresponding submaps
      submap_loop_closures_.add(
          gtsam::BetweenFactor<gtsam::Pose3>(from_key, to_key, T_s1_s2, noise));
    }
  }
  return incremental_pub;
}

void DistributedLoopClosure::processOptimizedPath(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses.empty()) {
    return;
  }
  // Store the optimized poses from dpgo in the submap atlas
  for (int submap_id = 0; submap_id < msg->poses.size(); ++submap_id) {
    const auto T_dpgo_submap = RosPoseToGtsam(msg->poses[submap_id].pose);
    const auto T_world_submap = T_world_dpgo_.compose(T_dpgo_submap);
    const auto submap = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id));
    submap->setPoseInWorldFrame(T_world_submap);
  }
  // Update the new (unoptimized) poses in the submap atlas by propagating odometry
  for (int submap_id = msg->poses.size(); submap_id < submap_atlas_->numSubmaps();
       ++submap_id) {
    const auto submap_curr = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id));
    const auto submap_prev = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id - 1));
    const auto T_odom_curr = submap_curr->getPoseInOdomFrame();
    const auto T_odom_prev = submap_prev->getPoseInOdomFrame();
    const auto T_prev_curr = T_odom_prev.inverse() * T_odom_curr;
    const auto T_world_curr = submap_prev->getPoseInWorldFrame() * T_prev_curr;
    submap_curr->setPoseInWorldFrame(T_world_curr);
  }
  backend_update_count_++;
  LOG(INFO) << "Received DPGO updates (current count: " << backend_update_count_
            << ").";
}

void DistributedLoopClosure::computePosesInWorldFrame(
    gtsam::Values::shared_ptr nodes) const {
  nodes->clear();
  // Using the optimized submap poses from dpgo, recover optimized poses for the
  // original VIO keyframes
  for (int submap_id = 0; submap_id < submap_atlas_->numSubmaps(); ++submap_id) {
    const auto submap = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id));
    const auto T_world_submap = submap->getPoseInWorldFrame();
    for (const int keyframe_id : submap->getKeyframeIDs()) {
      const auto keyframe = CHECK_NOTNULL(submap->getKeyframe(keyframe_id));
      const auto T_submap_keyframe = keyframe->getPoseInSubmapFrame();
      const auto T_world_keyframe = T_world_submap * T_submap_keyframe;
      nodes->insert(keyframe_id, T_world_keyframe);
    }
  }
}

void DistributedLoopClosure::savePosesToFile(const std::string& filename,
                                             const gtsam::Values& nodes) const {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    LOG(ERROR) << "Error opening log file: " << filename;
    return;
  }
  file << std::fixed << std::setprecision(8);
  file << "ns,pose_index,qx,qy,qz,qw,tx,ty,tz\n";

  for (const auto& key_pose : nodes) {
    gtsam::Quaternion quat =
        nodes.at<gtsam::Pose3>(key_pose.key).rotation().toQuaternion();
    gtsam::Point3 point = nodes.at<gtsam::Pose3>(key_pose.key).translation();
    const auto keyframe = submap_atlas_->getKeyframe(key_pose.key);
    file << keyframe->stamp() << ",";
    file << keyframe->id() << ",";
    file << quat.x() << ",";
    file << quat.y() << ",";
    file << quat.z() << ",";
    file << quat.w() << ",";
    file << point.x() << ",";
    file << point.y() << ",";
    file << point.z() << "\n";
  }
  file.close();
}

void DistributedLoopClosure::saveSubmapAtlas(const std::string& directory) const {
  // Store keyframes in their respective submap frames
  std::string keyframe_path = directory + "kimera_distributed_keyframes.csv";
  std::ofstream keyframe_file;
  keyframe_file.open(keyframe_path);
  if (!keyframe_file.is_open()) {
    LOG(ERROR) << "Error opening log file: " << keyframe_path;
    return;
  }
  keyframe_file << std::fixed << std::setprecision(8);
  keyframe_file << "keyframe_stamp_ns,keyframe_id,submap_id,qx,qy,qz,qw,tx,ty,tz\n";

  for (int submap_id = 0; submap_id < submap_atlas_->numSubmaps(); ++submap_id) {
    const auto submap = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id));
    for (const int keyframe_id : submap->getKeyframeIDs()) {
      const auto keyframe = CHECK_NOTNULL(submap->getKeyframe(keyframe_id));
      const auto T_submap_keyframe = keyframe->getPoseInSubmapFrame();
      // Save to log
      gtsam::Quaternion quat = T_submap_keyframe.rotation().toQuaternion();
      gtsam::Point3 point = T_submap_keyframe.translation();
      keyframe_file << keyframe->stamp() << ",";
      keyframe_file << keyframe_id << ",";
      keyframe_file << submap_id << ",";
      keyframe_file << quat.x() << ",";
      keyframe_file << quat.y() << ",";
      keyframe_file << quat.z() << ",";
      keyframe_file << quat.w() << ",";
      keyframe_file << point.x() << ",";
      keyframe_file << point.y() << ",";
      keyframe_file << point.z() << "\n";
    }
  }
  keyframe_file.close();

  // Store submaps in the robot's local odometry frame
  std::string submap_path = directory + "kimera_distributed_submaps.csv";
  std::ofstream submap_file;
  submap_file.open(submap_path);
  if (!submap_file.is_open()) {
    LOG(ERROR) << "Error opening log file: " << submap_path;
    return;
  }
  submap_file << std::fixed << std::setprecision(8);
  submap_file << "submap_stamp_ns,submap_id,qx,qy,qz,qw,tx,ty,tz\n";
  for (int submap_id = 0; submap_id < submap_atlas_->numSubmaps(); ++submap_id) {
    const auto submap = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id));
    gtsam::Quaternion quat = submap->getPoseInOdomFrame().rotation().toQuaternion();
    gtsam::Point3 point = submap->getPoseInOdomFrame().translation();
    submap_file << submap->stamp() << ",";
    submap_file << submap_id << ",";
    submap_file << quat.x() << ",";
    submap_file << quat.y() << ",";
    submap_file << quat.z() << ",";
    submap_file << quat.w() << ",";
    submap_file << point.x() << ",";
    submap_file << point.y() << ",";
    submap_file << point.z() << "\n";
  }
  submap_file.close();
}

bool DistributedLoopClosure::queryBowVectorsRequest(
    lcd::RobotId& selected_robot_id,
    std::set<lcd::PoseId>& bow_vectors) {
  // Form BoW vectors that are missing from each robot
  std::map<lcd::RobotId, std::set<lcd::PoseId>> missing_bow_vectors;
  for (lcd::RobotId robot_id = config_.my_id_; robot_id < config_.num_robots_;
       ++robot_id) {
    const auto& received_pose_ids = bow_received_.at(robot_id);
    const lcd::PoseId latest_pose_id = bow_latest_.at(robot_id);
    for (lcd::PoseId pose_id = 0; pose_id < latest_pose_id;
         pose_id += config_.bow_skip_num_) {
      if (received_pose_ids.find(pose_id) == received_pose_ids.end()) {
        if (robot_id == config_.my_id_) {
          // Missing BoW from myself.
          // This should not happen we should receive BoW directly from VIO
          LOG(ERROR) << "Robot " << robot_id << " cannot find BoW of itself! Missing "
                     << pose_id << " (latest = " << latest_pose_id << ").";
        } else {
          // Push to missing bow_vectors
          if (missing_bow_vectors.find(robot_id) == missing_bow_vectors.end())
            missing_bow_vectors[robot_id] = std::set<lcd::PoseId>();
          missing_bow_vectors[robot_id].emplace(pose_id);
        }
      }
    }
  }

  // Select robot with largest queue size
  bow_backlog_ = 0;
  selected_robot_id = 0;
  size_t selected_queue_size = 0;
  for (const auto& it : missing_bow_vectors) {
    lcd::RobotId robot_id = it.first;
    size_t robot_queue_size = it.second.size();
    bow_backlog_ += robot_queue_size;
    LOG(WARNING) << "Missing " << robot_queue_size << " bow vectors from robot "
                 << robot_id << ".";
    if (robot_connected_.at(robot_id) && robot_queue_size >= selected_queue_size) {
      selected_queue_size = robot_queue_size;
      selected_robot_id = robot_id;
    }
  }
  if (selected_queue_size == 0) {
    return false;
  }

  bow_vectors = missing_bow_vectors[selected_robot_id];
  return true;
}

bool DistributedLoopClosure::queryBowVectorsPublish(lcd::RobotId& selected_robot_id,
                                                    lcd::RobotPoseIdSet& bow_vectors) {
  std::unique_lock<std::mutex> requested_bows_lock(requested_bows_mutex_);

  // Select the robot with the largest queue size
  selected_robot_id = 0;
  size_t selected_queue_size = 0;
  for (const auto& it : requested_bows_) {
    lcd::RobotId robot_id = it.first;
    size_t robot_queue_size = it.second.size();
    if (robot_connected_[robot_id] && robot_queue_size >= selected_queue_size) {
      selected_robot_id = robot_id;
      selected_queue_size = robot_queue_size;
    }
  }

  if (selected_queue_size == 0) {
    return false;
  }

  std::set<lcd::PoseId>& requested_bows_from_robot =
      requested_bows_.at(selected_robot_id);
  auto it = requested_bows_from_robot.begin();
  while (true) {
    if (bow_vectors.size() >= config_.bow_batch_size_) {
      break;
    }

    if (it == requested_bows_from_robot.end()) {
      break;
    }

    lcd::PoseId pose_id = *it;
    it = requested_bows_from_robot.erase(
        it);  // remove the current ID and proceed to next one
    lcd::RobotPoseId requested_robot_pose_id(config_.my_id_, pose_id);
    if (!lcd_->bowExists(requested_robot_pose_id)) {
      LOG(ERROR) << "Requested BoW of frame " << pose_id << " does not exist!";
      continue;
    }
    bow_vectors.insert(requested_robot_pose_id);
  }
  return true;
}

void DistributedLoopClosure::queryFramesRequest(
    lcd::RobotPoseIdSet& local_vertex_ids,
    lcd::RobotId& selected_robot_id,
    lcd::RobotPoseIdSet& selected_vertex_ids) const {
  std::unordered_map<lcd::RobotId, lcd::RobotPoseIdSet> vertex_ids_map;
  for (const auto robot_queue : candidate_lc_) {
    // Form list of vertex ids that needs to be requested
    for (const auto& cand : robot_queue.second) {
      if (!lcd_->frameExists(cand.vertex_src_)) {
        const size_t& robot_id = cand.vertex_src_.first;
        if (vertex_ids_map.count(robot_id) == 0) {
          vertex_ids_map[robot_id] = lcd::RobotPoseIdSet();
        }
        vertex_ids_map.at(robot_id).emplace(cand.vertex_src_);
      }
      if (!lcd_->frameExists(cand.vertex_dst_)) {
        const size_t& robot_id = cand.vertex_dst_.first;
        if (vertex_ids_map.count(robot_id) == 0) {
          vertex_ids_map[robot_id] = lcd::RobotPoseIdSet();
        }
        vertex_ids_map.at(robot_id).emplace(cand.vertex_dst_);
      }
    }
  }

  // Process missing VLC frames of myself
  if (vertex_ids_map.count(config_.my_id_) &&
      !vertex_ids_map.at(config_.my_id_).empty()) {
    local_vertex_ids = vertex_ids_map.at(config_.my_id_);
  }

  // Select a peer robot with most missing frames to send request
  selected_robot_id = 0;
  size_t selected_queue_size = 0;
  for (const auto& it : vertex_ids_map) {
    lcd::RobotId robot_id = it.first;
    size_t robot_queue_size = it.second.size();
    if (robot_id == config_.my_id_ || !robot_connected_.at(robot_id)) {
      continue;
    }
    if (robot_queue_size >= selected_queue_size) {
      selected_queue_size = robot_queue_size;
      selected_robot_id = robot_id;
    }
  }

  if (selected_queue_size > 0) {
    selected_vertex_ids = vertex_ids_map.at(selected_robot_id);
  }
}

void DistributedLoopClosure::queryFramesPublish(
    lcd::RobotId& selected_robot_id,
    lcd::RobotPoseIdSet& selected_vertex_ids) {
  std::unique_lock<std::mutex> requested_frames_lock(requested_frames_mutex_);

  // Select the robot with the largest queue size
  selected_robot_id = 0;
  size_t selected_queue_size = 0;
  for (const auto& it : requested_frames_) {
    lcd::RobotId robot_id = it.first;
    size_t robot_queue_size = it.second.size();
    if (robot_connected_[robot_id] && robot_queue_size >= selected_queue_size) {
      selected_robot_id = robot_id;
      selected_queue_size = robot_queue_size;
    }
  }
  // ROS_INFO("Maximum num of VLC waiting: %zu (robot %zu).", selected_queue_size,
  // selected_robot_id);

  if (selected_queue_size > 0) {
    // Send VLC frames to the selected robot
    std::set<lcd::PoseId>& requested_frames_from_robot =
        requested_frames_.at(selected_robot_id);
    auto it = requested_frames_from_robot.begin();
    while (true) {
      if (selected_vertex_ids.size() >= config_.vlc_batch_size_) {
        break;
      }
      if (it == requested_frames_from_robot.end()) {
        break;
      }
      lcd::RobotPoseId vertex_id(config_.my_id_, *it);
      if (lcd_->frameExists(vertex_id)) {
        selected_vertex_ids.insert(vertex_id);
      }
      it = requested_frames_from_robot.erase(
          it);  // remove the current ID and proceed to next one
    }
  }
}

void DistributedLoopClosure::detectLoopSpin() {
  std::unique_lock<std::mutex> bow_lock(bow_msgs_mutex_);
  auto it = bow_msgs_.begin();
  int num_detection_performed = 0;

  while (num_detection_performed < config_.detection_batch_size_) {
    if (it == bow_msgs_.end()) {
      break;
    }
    const pose_graph_tools::BowQuery msg = *it;
    lcd::RobotId query_robot = msg.robot_id;
    lcd::PoseId query_pose = msg.pose_id;
    lcd::RobotPoseId query_vertex(query_robot, query_pose);
    DBoW2::BowVector bow_vec;
    kimera_multi_lcd::BowVectorFromMsg(msg.bow_vector, &bow_vec);

    if (query_pose <= 2 * config_.bow_skip_num_) {
      // We cannot detect loop for the very first few frames
      // Remove and proceed to next message
      LOG(INFO) << "Received initial BoW from robot " << query_robot << " (pose "
                << query_pose << ").";
      lcd_->addBowVector(query_vertex, bow_vec);
      it = bow_msgs_.erase(it);
    } else if (!lcd_->findPreviousBoWVector(query_vertex)) {
      // We cannot detect loop since the BoW of previous frame is missing
      // (Recall we need that to compute nss factor)
      // In this case we skip the message and try again later
      // ROS_WARN("Cannot detect loop for (%zu,%zu) because previous BoW is missing.",
      //           query_robot, query_pose);
      ++it;
    } else {
      // We are ready to detect loop for this query message
      // ROS_INFO("Detect loop for (%zu,%zu).", query_robot, query_pose);
      detectLoop(query_vertex, bow_vec);
      num_detection_performed++;
      // Inter-robot queries will count as communication payloads
      if (query_robot != config_.my_id_) {
        received_bow_bytes_.push_back(
            kimera_multi_lcd::computeBowQueryPayloadBytes(msg));
      }
      lcd_->addBowVector(query_vertex, bow_vec);
      it = bow_msgs_.erase(it);  // Erase this message and move on to next one
    }
  }
  // if (!bow_msgs_.empty()) {
  // ROS_INFO("Performed %d loop detection (%zu pending).", num_detection_performed,
  // bow_msgs_.size());
  //}
}

void DistributedLoopClosure::detectLoop(const lcd::RobotPoseId& vertex_query,
                                        const DBoW2::BowVector& bow_vec) {
  const lcd::RobotId robot_query = vertex_query.first;
  const lcd::PoseId pose_query = vertex_query.second;
  std::vector<lcd::RobotPoseId> vertex_matches;
  std::vector<double> match_scores;
  {  // start lcd critical section
    std::unique_lock<std::mutex> lcd_lock(lcd_mutex_);

    // Incoming bow vector is from my trajectory
    // Detect loop closures with all robots in the database
    // (including myself if inter_robot_only is set to false)
    if (robot_query == config_.my_id_) {
      if (lcd_->detectLoop(vertex_query, bow_vec, &vertex_matches, &match_scores)) {
        for (size_t i = 0; i < vertex_matches.size(); ++i) {
          lcd::PotentialVLCEdge potential_edge(
              vertex_query, vertex_matches[i], match_scores[i]);

          {  // start candidate critical section. Add to candidate for request
            std::unique_lock<std::mutex> candidate_lock(candidate_lc_mutex_);
            candidate_lc_.at(robot_query).push_back(potential_edge);
          }  // end candidate critical section
        }
      }
    }

    // Incoming bow vector is from another robot
    // Detect loop closures ONLY with my trajectory
    if (robot_query != config_.my_id_) {
      if (lcd_->detectLoopWithRobot(
              config_.my_id_, vertex_query, bow_vec, &vertex_matches, &match_scores)) {
        for (size_t i = 0; i < vertex_matches.size(); ++i) {
          lcd::PotentialVLCEdge potential_edge(
              vertex_query, vertex_matches[i], match_scores[i]);

          {
            // start candidate critical section. Add to candidate for request
            std::unique_lock<std::mutex> candidate_lock(candidate_lc_mutex_);
            candidate_lc_.at(robot_query).push_back(potential_edge);
          }  // end candidate critical section
        }
      }
    }
  }  // end lcd critical section
}

void DistributedLoopClosure::verifyLoopSpin() {
  while (queued_lc_.size() > 0) {
    // Attempt to detect a single loop closure
    lcd::PotentialVLCEdge potential_edge = queued_lc_.front();
    const auto& vertex_query = potential_edge.vertex_src_;
    const auto& vertex_match = potential_edge.vertex_dst_;
    const double match_score = potential_edge.score_;

    {  // start lcd critical section
      std::unique_lock<std::mutex> lcd_lock(lcd_mutex_);
      // Both frames should already exist locally
      CHECK(lcd_->frameExists(vertex_query));
      CHECK(lcd_->frameExists(vertex_match));

      // Find correspondences between frames.
      std::vector<unsigned int> i_query, i_match;
      lcd_->computeMatchedIndices(vertex_query, vertex_match, &i_query, &i_match);
      assert(i_query.size() == i_match.size());

      // Geometric verification
      gtsam::Rot3 monoR_query_match;
      gtsam::Pose3 T_query_match;
      // Perform monocular RANSAC
      if (lcd_->geometricVerificationNister(
              vertex_query, vertex_match, &i_query, &i_match, &monoR_query_match)) {
        size_t mono_inliers_count = i_query.size();

        // Perform stereo RANSAC, using relative rotation estimate from mono RANSAC as
        // prior
        if (lcd_->recoverPose(vertex_query,
                              vertex_match,
                              &i_query,
                              &i_match,
                              &T_query_match,
                              &monoR_query_match)) {
          size_t stereo_inliers_count = i_query.size();
          // Get loop closure between keyframes (for logging purpose)
          lcd::VLCEdge keyframe_edge(vertex_query, vertex_match, T_query_match);
          keyframe_edge.normalized_bow_score_ = match_score;
          keyframe_edge.mono_inliers_ = mono_inliers_count;
          keyframe_edge.stereo_inliers_ = stereo_inliers_count;
          keyframe_edge.stamp_ns_ = ros::Time::now().toNSec();
          const auto frame1 = lcd_->getVLCFrame(vertex_query);
          const auto frame2 = lcd_->getVLCFrame(vertex_match);
          gtsam::Symbol keyframe_from(robot_id_to_prefix.at(frame1.robot_id_),
                                      frame1.pose_id_);
          gtsam::Symbol keyframe_to(robot_id_to_prefix.at(frame2.robot_id_),
                                    frame2.pose_id_);
          static const gtsam::SharedNoiseModel& noise =
              gtsam::noiseModel::Isotropic::Variance(6, 1e-2);

          // Log loop closure between keyframes
          keyframe_loop_closures_.add(gtsam::BetweenFactor<gtsam::Pose3>(
              keyframe_from, keyframe_to, T_query_match, noise));
          logLoopClosure(keyframe_edge);
          num_inter_robot_loops_++;
          lcd::RobotId other_robot = 0;
          if (vertex_query.first == config_.my_id_) {
            other_robot = vertex_match.first;
          } else {
            other_robot = vertex_query.first;
          }
          num_loops_with_robot_[other_robot] += 1;

          // Get loop closure between the corresponding two submaps
          const auto T_s1_f1 = frame1.T_submap_pose_;
          const auto T_s2_f2 = frame2.T_submap_pose_;
          const auto T_f1_f2 = T_query_match;
          const auto T_s1_s2 = T_s1_f1 * T_f1_f2 * (T_s2_f2.inverse());
          gtsam::Symbol submap_from(robot_id_to_prefix.at(frame1.robot_id_),
                                    frame1.submap_id_);
          gtsam::Symbol submap_to(robot_id_to_prefix.at(frame2.robot_id_),
                                  frame2.submap_id_);

          // Skip intra loop connecting identical or consecutive submaps from the same
          // robot
          const int robot_id_from = frame1.robot_id_;
          const int robot_id_to = frame2.robot_id_;
          const int submap_id_from = frame1.submap_id_;
          const int submap_id_to = frame2.submap_id_;
          if (robot_id_from == robot_id_to && abs(submap_id_from - submap_id_to) <= 1) {
            continue;
          }

          // Add this loop closure if no loop closure exists between the two submaps
          // This ensures that there is at most one loop closure between every pair of
          // submaps
          lcd::EdgeID submap_edge_id(
              frame1.robot_id_, frame1.submap_id_, frame2.robot_id_, frame2.submap_id_);
          bool loop_exists = (submap_loop_closures_ids_.find(submap_edge_id) !=
                              submap_loop_closures_ids_.end());
          if (!loop_exists) {
            submap_loop_closures_ids_.emplace(submap_edge_id);
            // Add new loop to queue for synchronization with other robots
            submap_loop_closures_queue_[submap_edge_id] =
                gtsam::BetweenFactor<gtsam::Pose3>(
                    submap_from, submap_to, T_s1_s2, noise);
            // submap_loop_closures_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            // submap_from, submap_to, T_s1_s2, noise));
          }
          LOG(INFO) << "Verified loop (" << vertex_query.first << ","
                    << vertex_query.second << ")-(" << vertex_match.first << ","
                    << vertex_match.second << "). Normalized BoW score: " << match_score
                    << ". Mono inliers : " << mono_inliers_count
                    << ". Stereo inliers : " << stereo_inliers_count << ".";
        }
      }
    }  // end lcd critical section
    queued_lc_.pop();
  }
}

pose_graph_tools::PoseGraph DistributedLoopClosure::getSubmapPoseGraph(
    bool incremental) {
  // Start submap critical section
  // std::unique_lock<std::mutex> submap_lock(submap_atlas_mutex_);
  // Fill in submap-level loop closures
  pose_graph_tools::PoseGraph out_graph;

  if (submap_atlas_->numSubmaps() == 0) {
    return out_graph;
  }

  if (incremental) {
    gtsam::NonlinearFactorGraph new_loop_closures(
        submap_loop_closures_.begin() + last_get_lc_idx_, submap_loop_closures_.end());
    out_graph = GtsamGraphToRos(new_loop_closures, gtsam::Values());
    last_get_lc_idx_ = submap_loop_closures_.size();
  } else {
    out_graph = GtsamGraphToRos(submap_loop_closures_, gtsam::Values());
  }
  const std::string robot_name = config_.robot_names_.at(config_.my_id_);

  // Fill in submap-level odometry
  size_t start_idx = incremental ? last_get_submap_idx_ : 0;
  size_t end_idx = submap_atlas_->numSubmaps() - 1;
  for (int submap_id = start_idx; submap_id < end_idx; ++submap_id) {
    pose_graph_tools::PoseGraphEdge edge;
    const auto submap_src = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id));
    const auto submap_dst = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id + 1));
    const auto T_odom_src = submap_src->getPoseInOdomFrame();
    const auto T_odom_dst = submap_dst->getPoseInOdomFrame();
    const auto T_src_dst = (T_odom_src.inverse()) * T_odom_dst;
    edge.robot_from = config_.my_id_;
    edge.robot_to = config_.my_id_;
    edge.key_from = submap_src->id();
    edge.key_to = submap_dst->id();
    edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
    edge.pose = GtsamPoseToRos(T_src_dst);
    edge.header.stamp.fromNSec(submap_dst->stamp());
    // TODO(yun) add frame id param
    edge.header.frame_id = config_.frame_id_;
    out_graph.edges.push_back(edge);
  }

  // Fill in submap nodes
  start_idx = (incremental) ? start_idx + 1 : start_idx;
  for (int submap_id = start_idx; submap_id < end_idx + 1; ++submap_id) {
    pose_graph_tools::PoseGraphNode node;
    const auto submap = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id));
    node.robot_id = config_.my_id_;
    node.key = submap->id();
    node.header.stamp.fromNSec(submap->stamp());
    node.header.frame_id = config_.frame_id_;
    node.pose = GtsamPoseToRos(submap->getPoseInOdomFrame());
    out_graph.nodes.push_back(node);
    out_graph.header.stamp.fromNSec(submap->stamp());
    out_graph.header.frame_id = config_.frame_id_;
  }

  if (incremental) {
    last_get_submap_idx_ = end_idx;
  }

  return out_graph;
}

void DistributedLoopClosure::processInternalVLC(
    const pose_graph_tools::VLCFramesConstPtr& msg) {
  for (const auto& frame_msg : msg->frames) {
    lcd::VLCFrame frame;
    kimera_multi_lcd::VLCFrameFromMsg(frame_msg, &frame);
    if (frame.robot_id_ != config_.my_id_) {
      continue;
    }

    // Fill in submap information for this keyframe
    const auto keyframe = submap_atlas_->getKeyframe(frame.pose_id_);
    if (!keyframe) {
      LOG(WARNING) << "Received internal VLC frame " << frame.pose_id_
                   << " but submap info is not found.";
      continue;
    }
    frame.submap_id_ = CHECK_NOTNULL(keyframe->getSubmap())->id();
    frame.T_submap_pose_ = keyframe->getPoseInSubmapFrame();

    // Store new frame
    lcd::RobotPoseId vertex_id(frame.robot_id_, frame.pose_id_);
    {  // start lcd critical section
      std::unique_lock<std::mutex> lcd_lock(lcd_mutex_);
      lcd_->addVLCFrame(vertex_id, frame);
    }  // end lcd critical section
  }
}

size_t DistributedLoopClosure::updateCandidateList() {
  // return total number of candidates still missing VLC frames
  size_t total_candidates = 0;
  size_t ready_candidates = 0;
  // Recompute backlog on missing VLC frames
  vlc_backlog_ = 0;
  // start candidate list critical section
  std::unique_lock<std::mutex> candidate_lock(candidate_lc_mutex_);
  for (const auto& robot_queue : candidate_lc_) {
    // Create new vector of candidates still missing VLC frames
    std::vector<lcd::PotentialVLCEdge> unresolved_candidates;
    for (const auto& candidate : robot_queue.second) {
      if (!lcd_->frameExists(candidate.vertex_src_)) {
        vlc_backlog_++;
      }
      if (!lcd_->frameExists(candidate.vertex_dst_)) {
        vlc_backlog_++;
      }
      if (lcd_->frameExists(candidate.vertex_src_) &&
          lcd_->frameExists(candidate.vertex_dst_)) {
        queued_lc_.push(candidate);
        ready_candidates++;
      } else {
        unresolved_candidates.push_back(candidate);
        total_candidates++;
      }
    }
    // Update candidate list
    candidate_lc_[robot_queue.first] = unresolved_candidates;
  }
  LOG(INFO) << "Loop closure candidates ready for verification: " << ready_candidates
            << ", waiting for frames: " << total_candidates;
  return total_candidates;
}

gtsam::Pose3 DistributedLoopClosure::getOdomInWorldFrame() const {
  gtsam::Pose3 T_world_odom = gtsam::Pose3();
  const auto keyframe = submap_atlas_->getLatestKeyframe();
  if (keyframe) {
    const auto submap = keyframe->getSubmap();
    CHECK_NOTNULL(submap);
    const gtsam::Pose3 T_odom_kf = keyframe->getPoseInOdomFrame();
    const gtsam::Pose3 T_submap_kf = keyframe->getPoseInSubmapFrame();
    const gtsam::Pose3 T_world_submap = submap->getPoseInWorldFrame();
    const gtsam::Pose3 T_world_kf = T_world_submap * T_submap_kf;
    T_world_odom = T_world_kf * (T_odom_kf.inverse());
  }
  return T_world_odom;
}

gtsam::Pose3 DistributedLoopClosure::getLatestKFInWorldFrame() const {
  gtsam::Pose3 T_world_kf = gtsam::Pose3();
  const auto keyframe = submap_atlas_->getLatestKeyframe();
  if (keyframe) {
    const auto submap = keyframe->getSubmap();
    CHECK_NOTNULL(submap);
    const gtsam::Pose3 T_submap_kf = keyframe->getPoseInSubmapFrame();
    const gtsam::Pose3 T_world_submap = submap->getPoseInWorldFrame();
    T_world_kf = T_world_submap * T_submap_kf;
  }
  return T_world_kf;
}

gtsam::Pose3 DistributedLoopClosure::getLatestKFInOdomFrame() const {
  gtsam::Pose3 T_odom_kf = gtsam::Pose3();
  const auto keyframe = submap_atlas_->getLatestKeyframe();
  if (keyframe) {
    T_odom_kf = keyframe->getPoseInOdomFrame();
  }
  return T_odom_kf;
}

void DistributedLoopClosure::saveBowVectors(const std::string& filepath) const {
  for (const auto& robot_pose_id : bow_latest_) {
    std::string bow_vector_save = filepath + "/robot_" +
                                  std::to_string(robot_pose_id.first) +
                                  "_bow_vectors.json";
    LOG(INFO) << "Saving loop closure BoWs to " << bow_vector_save;
    lcd::saveBowVectors(lcd_->getBoWVectors(robot_pose_id.first), bow_vector_save);
  }
}

void DistributedLoopClosure::saveVLCFrames(const std::string& filepath) const {
  for (const auto& robot_pose_id : bow_latest_) {
    std::string vlc_frames_save =
        filepath + "/robot_" + std::to_string(robot_pose_id.first) + "_vlc_frames.json";
    LOG(INFO) << "Saving loop closure Frames to " << vlc_frames_save;
    lcd::saveVLCFrames(lcd_->getVLCFrames(robot_pose_id.first), vlc_frames_save);
  }
}

void DistributedLoopClosure::loadBowVectors(size_t robot_id,
                                            const std::string& bow_json) {
  ROS_INFO_STREAM("Loading BoW vectors from " << bow_json);
  std::map<lcd::PoseId, DBoW2::BowVector> bow_vectors;
  lcd::loadBowVectors(bow_json, bow_vectors);
  ROS_INFO_STREAM("Loaded " << bow_vectors.size() << " BoW vectors.");
  bow_latest_[robot_id] = 0;
  bow_received_[robot_id] = std::unordered_set<lcd::PoseId>();
  for (const auto& id_bow : bow_vectors) {
    lcd::RobotPoseId id(robot_id, id_bow.first);
    lcd_->addBowVector(id, id_bow.second);
    bow_latest_[robot_id] = id_bow.first;
    bow_received_[robot_id].insert(id_bow.first);
  }
}

void DistributedLoopClosure::loadVLCFrames(size_t robot_id,
                                           const std::string& vlc_json) {
  ROS_INFO_STREAM("Loading VLC frames from " << vlc_json);
  std::map<lcd::PoseId, lcd::VLCFrame> vlc_frames;
  lcd::loadVLCFrames(vlc_json, vlc_frames);
  ROS_INFO_STREAM("Loaded " << vlc_frames.size() << " VLC frames.");

  for (const auto& id_vlc : vlc_frames) {
    lcd::RobotPoseId id(robot_id, id_vlc.first);
    lcd_->addVLCFrame(id, id_vlc.second);
  }
}

void DistributedLoopClosure::createLogFiles() {
  std::string pose_file_path = config_.log_output_dir_ + "odometry_poses.csv";
  std::string inter_lc_file_path = config_.log_output_dir_ + "loop_closures.csv";
  std::string lcd_file_path = config_.log_output_dir_ + "lcd_log.csv";

  odometry_file_.open(pose_file_path);
  if (!odometry_file_.is_open())
    LOG(ERROR) << "Error opening log file: " << pose_file_path;
  odometry_file_ << std::fixed << std::setprecision(15);
  odometry_file_ << "stamp_ns,robot_index,pose_index,qx,qy,qz,qw,tx,ty,tz\n";
  odometry_file_.flush();

  loop_closure_file_.open(inter_lc_file_path);
  if (!loop_closure_file_.is_open())
    LOG(ERROR) << "Error opening log file: " << inter_lc_file_path;
  loop_closure_file_ << std::fixed << std::setprecision(15);
  loop_closure_file_ << "robot1,pose1,robot2,pose2,qx,qy,qz,qw,tx,ty,tz,norm_bow_score,"
                        "mono_inliers,stereo_inliers,stamp_ns\n";
  loop_closure_file_.flush();

  lcd_log_file_.open(lcd_file_path);
  if (!lcd_log_file_.is_open()) {
    LOG(ERROR) << "Error opening log file: " << lcd_file_path;
  }
  lcd_log_file_ << std::fixed << std::setprecision(15);
  lcd_log_file_ << "stamp_ns, bow_matches, mono_verifications, stereo_verifications, "
                   "num_loop_closures, bow_bytes, vlc_bytes, bow_backlog, vlc_backlog, "
                   "num_loops_with_robots\n";
  lcd_log_file_.flush();
}

void DistributedLoopClosure::closeLogFiles() {
  if (odometry_file_.is_open()) odometry_file_.close();
  if (loop_closure_file_.is_open()) loop_closure_file_.close();
  if (lcd_log_file_.is_open()) {
    lcd_log_file_.close();
  }
}

void DistributedLoopClosure::logLcdStat() {
  if (lcd_log_file_.is_open()) {
    // auto elapsed_sec = (ros::Time::now() - start_time_).toSec();
    lcd_log_file_ << ros::Time::now().toNSec() << ",";
    lcd_log_file_ << lcd_->totalBoWMatches() << ",";
    lcd_log_file_ << lcd_->getNumGeomVerificationsMono() << ",";
    lcd_log_file_ << lcd_->getNumGeomVerifications() << ",";
    lcd_log_file_ << keyframe_loop_closures_.size() << ",";
    lcd_log_file_ << std::accumulate(
                         received_bow_bytes_.begin(), received_bow_bytes_.end(), 0)
                  << ",";
    lcd_log_file_ << std::accumulate(
                         received_vlc_bytes_.begin(), received_vlc_bytes_.end(), 0)
                  << ",";
    lcd_log_file_ << bow_backlog_ << ",";
    lcd_log_file_ << vlc_backlog_ << ",";
    for (size_t robot_id = 0; robot_id < config_.num_robots_; ++robot_id) {
      lcd_log_file_ << num_loops_with_robot_[robot_id] << ",";
    }
    lcd_log_file_ << "\n";
    lcd_log_file_.flush();
  }
}

void DistributedLoopClosure::logOdometryPose(const gtsam::Symbol& symbol_frame,
                                             const gtsam::Pose3& T_odom_frame,
                                             uint64_t ts) {
  if (odometry_file_.is_open()) {
    gtsam::Quaternion quat = T_odom_frame.rotation().toQuaternion();
    gtsam::Point3 point = T_odom_frame.translation();
    const uint32_t robot_id = robot_prefix_to_id.at(symbol_frame.chr());
    const uint32_t frame_id = symbol_frame.index();
    odometry_file_ << ts << ",";
    odometry_file_ << robot_id << ",";
    odometry_file_ << frame_id << ",";
    odometry_file_ << quat.x() << ",";
    odometry_file_ << quat.y() << ",";
    odometry_file_ << quat.z() << ",";
    odometry_file_ << quat.w() << ",";
    odometry_file_ << point.x() << ",";
    odometry_file_ << point.y() << ",";
    odometry_file_ << point.z() << "\n";
    odometry_file_.flush();
  }
}

void DistributedLoopClosure::logLoopClosure(const lcd::VLCEdge& keyframe_edge) {
  if (loop_closure_file_.is_open()) {
    gtsam::Quaternion quat = keyframe_edge.T_src_dst_.rotation().toQuaternion();
    gtsam::Point3 point = keyframe_edge.T_src_dst_.translation();
    const lcd::RobotId robot_src = keyframe_edge.vertex_src_.first;
    const lcd::PoseId frame_src = keyframe_edge.vertex_src_.second;
    const lcd::RobotId robot_dst = keyframe_edge.vertex_dst_.first;
    const lcd::PoseId frame_dst = keyframe_edge.vertex_dst_.second;
    loop_closure_file_ << robot_src << ",";
    loop_closure_file_ << frame_src << ",";
    loop_closure_file_ << robot_dst << ",";
    loop_closure_file_ << frame_dst << ",";
    loop_closure_file_ << quat.x() << ",";
    loop_closure_file_ << quat.y() << ",";
    loop_closure_file_ << quat.z() << ",";
    loop_closure_file_ << quat.w() << ",";
    loop_closure_file_ << point.x() << ",";
    loop_closure_file_ << point.y() << ",";
    loop_closure_file_ << point.z() << ",";
    loop_closure_file_ << keyframe_edge.normalized_bow_score_ << ",";
    loop_closure_file_ << keyframe_edge.mono_inliers_ << ",";
    loop_closure_file_ << keyframe_edge.stereo_inliers_ << ",";
    loop_closure_file_ << keyframe_edge.stamp_ns_ << "\n";
    loop_closure_file_.flush();
  }
}

void DistributedLoopClosure::loadOdometryFromFile(const std::string& pose_file) {
  std::ifstream infile(pose_file);
  if (!infile.is_open()) {
    LOG(ERROR) << "Could not open specified file!";
    ros::shutdown();
  }

  size_t num_poses_read = 0;

  // Scalars that will be filled
  uint64_t stamp_ns;
  uint32_t robot_id;
  uint32_t frame_id;
  double qx, qy, qz, qw;
  double tx, ty, tz;

  std::string line;
  std::string token;

  // Skip first line (headers)
  std::getline(infile, line);

  // Iterate over remaining lines
  while (std::getline(infile, line)) {
    std::istringstream ss(line);

    std::getline(ss, token, ',');
    stamp_ns = std::stoull(token);
    std::getline(ss, token, ',');
    robot_id = std::stoi(token);
    std::getline(ss, token, ',');
    frame_id = std::stoi(token);

    std::getline(ss, token, ',');
    qx = std::stod(token);
    std::getline(ss, token, ',');
    qy = std::stod(token);
    std::getline(ss, token, ',');
    qz = std::stod(token);
    std::getline(ss, token, ',');
    qw = std::stod(token);

    std::getline(ss, token, ',');
    tx = std::stod(token);
    std::getline(ss, token, ',');
    ty = std::stod(token);
    std::getline(ss, token, ',');
    tz = std::stod(token);

    // Add this keyframe to submap atlas
    CHECK_EQ(robot_id, config_.my_id_);
    gtsam::Pose3 T_odom_keyframe;
    T_odom_keyframe =
        gtsam::Pose3(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(tx, ty, tz));
    if (submap_atlas_->hasKeyframe(frame_id)) continue;
    if (frame_id != submap_atlas_->numKeyframes()) {
      LOG(ERROR) << "Received out of ordered keyframe. Expected id:"
                 << submap_atlas_->numKeyframes() << ", received=" << frame_id;
    }
    submap_atlas_->createKeyframe(frame_id, T_odom_keyframe, stamp_ns);
    num_poses_read++;
  }
  infile.close();
  LOG(INFO) << "Loaded " << num_poses_read << " from " << pose_file;
}

void DistributedLoopClosure::loadLoopClosuresFromFile(const std::string& lc_file) {
  std::ifstream infile(lc_file);
  if (!infile.is_open()) {
    LOG(ERROR) << "Could not open specified file!";
    ros::shutdown();
  }
  size_t num_measurements_read = 0;

  // Scalars that will be filled
  uint32_t robot_from, robot_to, pose_from, pose_to;
  double qx, qy, qz, qw;
  double tx, ty, tz;

  std::string line;
  std::string token;

  // Skip first line (headers)
  std::getline(infile, line);

  // Iterate over remaining lines
  while (std::getline(infile, line)) {
    std::istringstream ss(line);

    std::getline(ss, token, ',');
    robot_from = std::stoi(token);
    std::getline(ss, token, ',');
    pose_from = std::stoi(token);
    std::getline(ss, token, ',');
    robot_to = std::stoi(token);
    std::getline(ss, token, ',');
    pose_to = std::stoi(token);

    std::getline(ss, token, ',');
    qx = std::stod(token);
    std::getline(ss, token, ',');
    qy = std::stod(token);
    std::getline(ss, token, ',');
    qz = std::stod(token);
    std::getline(ss, token, ',');
    qw = std::stod(token);

    std::getline(ss, token, ',');
    tx = std::stod(token);
    std::getline(ss, token, ',');
    ty = std::stod(token);
    std::getline(ss, token, ',');
    tz = std::stod(token);

    gtsam::Symbol keyframe_from(robot_id_to_prefix.at(robot_from), pose_from);
    gtsam::Symbol keyframe_to(robot_id_to_prefix.at(robot_to), pose_to);
    gtsam::Pose3 T_f1_f2;
    T_f1_f2 = gtsam::Pose3(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(tx, ty, tz));
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(6, 1e-2);
    offline_keyframe_loop_closures_.add(
        gtsam::BetweenFactor<gtsam::Pose3>(keyframe_from, keyframe_to, T_f1_f2, noise));
    num_measurements_read++;
  }
  infile.close();
  LOG(INFO) << "Loaded " << num_measurements_read << " loop closures from " << lc_file;
}

void DistributedLoopClosure::processOfflineLoopClosures() {
  gtsam::NonlinearFactorGraph remaining_loop_closures_;
  size_t num_loops_processed = 0;
  for (const auto& factor : offline_keyframe_loop_closures_) {
    gtsam::Symbol front(factor->front());
    gtsam::Symbol back(factor->back());
    lcd::PoseId pose_from = front.index();
    lcd::PoseId pose_to = back.index();
    lcd::RobotId robot_from = robot_prefix_to_id.at(front.chr());
    lcd::RobotId robot_to = robot_prefix_to_id.at(back.chr());
    lcd::RobotPoseId vertex_from(robot_from, pose_from);
    lcd::RobotPoseId vertex_to(robot_to, pose_to);
    if (offline_robot_pose_msg_.find(vertex_from) != offline_robot_pose_msg_.end() &&
        offline_robot_pose_msg_.find(vertex_to) != offline_robot_pose_msg_.end()) {
      num_loops_processed++;
      const auto& msg_from = offline_robot_pose_msg_.at(vertex_from);
      const auto& msg_to = offline_robot_pose_msg_.at(vertex_to);
      const int submap_id_from = (int)msg_from.submap_id;
      const int submap_id_to = (int)msg_to.submap_id;
      const lcd::EdgeID submap_edge_id(
          robot_from, submap_id_from, robot_to, submap_id_to);
      // Skip if loop closure connects identical or consecutive submaps of same robots
      if (robot_from == robot_to && abs(submap_id_from - submap_id_to) <= 1) {
        LOG(WARNING) << "Skip intra loop connecting submap " << submap_id_from
                     << " and " << submap_id_to << ".";
        continue;
      }
      bool loop_exist = (submap_loop_closures_ids_.find(submap_edge_id) !=
                         submap_loop_closures_ids_.end());
      if (!loop_exist) {
        const gtsam::BetweenFactor<gtsam::Pose3>& factor_pose3 =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
        submap_loop_closures_ids_.emplace(submap_edge_id);
        // Extract relative transformation between submaps
        gtsam::Pose3 T_s1_f1 = RosPoseToGtsam(msg_from.T_submap_pose);
        gtsam::Pose3 T_s2_f2 = RosPoseToGtsam(msg_to.T_submap_pose);
        gtsam::Pose3 T_f1_f2 = factor_pose3.measured();
        gtsam::Pose3 T_s1_s2 = T_s1_f1 * T_f1_f2 * (T_s2_f2.inverse());
        // Add new loop to queue for synchronization with other robots
        gtsam::Symbol submap_from(robot_id_to_prefix.at(robot_from), submap_id_from);
        gtsam::Symbol submap_to(robot_id_to_prefix.at(robot_to), submap_id_to);
        static const gtsam::SharedNoiseModel& noise =
            gtsam::noiseModel::Isotropic::Variance(6, 1e-2);
        // submap_loop_closures_queue_[submap_edge_id] =
        // gtsam::BetweenFactor<gtsam::Pose3>(
        //     submap_from, submap_to, T_s1_s2, noise);
        submap_loop_closures_.add(
            gtsam::BetweenFactor<gtsam::Pose3>(submap_from, submap_to, T_s1_s2, noise));
      }
    } else {
      // Keyframe info is not yet available
      remaining_loop_closures_.add(factor);
    }
  }
  LOG(INFO) << "Processed " << num_loops_processed
            << " offline loop closures, remaining " << remaining_loop_closures_.size()
            << ".";
  offline_keyframe_loop_closures_ = remaining_loop_closures_;
}

}  // namespace kimera_distributed