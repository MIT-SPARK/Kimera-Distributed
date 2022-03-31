/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu) Yun Chang (yunchang@mit.edu)
 */

#include <kimera_distributed/DistributedLoopClosure.h>
#include <kimera_distributed/prefix.h>

#include <DBoW2/DBoW2.h>
#include <gtsam/geometry/Pose3.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/VLCFrameQuery.h>
#include <glog/logging.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

namespace kimera_distributed {

DistributedLoopClosure::DistributedLoopClosure(const ros::NodeHandle& n)
    : nh_(n),
      my_id_(0),
      num_robots_(1),
      log_output_(false),
      run_offline_(false),
      lcd_(new lcd::LoopClosureDetector),
      last_get_submap_idx_(0),
      last_get_lc_idx_(0) {
  int my_id_int = -1;
  int num_robots_int = -1;
  ros::param::get("~robot_id", my_id_int);
  ros::param::get("~num_robots", num_robots_int);
  ros::param::get("~frame_id", frame_id_);
  assert(my_id_int >= 0);
  assert(num_robots_int > 0);
  my_id_ = my_id_int;
  num_robots_ = num_robots_int;

  // Used for logging
  received_bow_bytes_.clear();
  received_vlc_bytes_.clear();

  // Path to log outputs
  log_output_ = ros::param::get("~log_output_path", log_output_dir_);
  ros::param::get("~run_offline", run_offline_);

  // Visual place recognition params
  ros::param::get("~alpha", lcd_params_.alpha_);
  ros::param::get("~dist_local", lcd_params_.dist_local_);
  ros::param::get("~max_db_results", lcd_params_.max_db_results_);
  ros::param::get("~min_nss_factor", lcd_params_.min_nss_factor_);

  // Lcd Third Party Wrapper Params
  ros::param::get("~max_nrFrames_between_islands",
                  lcd_params_.lcd_tp_params_.max_nrFrames_between_islands_);
  ros::param::get("~max_nrFrames_between_queries",
                  lcd_params_.lcd_tp_params_.max_nrFrames_between_queries_);
  ros::param::get("~max_intraisland_gap",
                  lcd_params_.lcd_tp_params_.max_intraisland_gap_);
  ros::param::get("~min_matches_per_island",
                  lcd_params_.lcd_tp_params_.min_matches_per_island_);
  ros::param::get("~min_temporal_matches",
                  lcd_params_.lcd_tp_params_.min_temporal_matches_);

  // Geometric verification params
  ros::param::get("~ransac_threshold_mono", lcd_params_.ransac_threshold_mono_);
  ros::param::get("~ransac_inlier_percentage_mono",
                  lcd_params_.ransac_inlier_percentage_mono_);
  ros::param::get("~max_ransac_iterations_mono",
                  lcd_params_.max_ransac_iterations_mono_);
  ros::param::get("~lowe_ratio", lcd_params_.lowe_ratio_);
  ros::param::get("~max_ransac_iterations", lcd_params_.max_ransac_iterations_);
  ros::param::get("~ransac_threshold", lcd_params_.ransac_threshold_);
  ros::param::get("~geometric_verification_min_inlier_count",
                  lcd_params_.geometric_verification_min_inlier_count_);
  ros::param::get("~geometric_verification_min_inlier_percentage",
                  lcd_params_.geometric_verification_min_inlier_percentage_);
  ros::param::get("~detect_interrobot_only", lcd_params_.inter_robot_only_);

  ros::param::get("~vocabulary_path", lcd_params_.vocab_path_);

  // Load VLC frame batch size
  vlc_batch_size_ = 25;
  ros::param::get("~vlc_batch_size", vlc_batch_size_);

  // Load robot names and initialize candidate lc queues
  for (size_t id = 0; id < num_robots_; id++) {
    std::string robot_name = "kimera" + std::to_string(id);
    ros::param::get("~robot" + std::to_string(id) + "_name", robot_name);
    robot_names_[id] = robot_name;

    candidate_lc_[id] = std::vector<lcd::PotentialVLCEdge>{};
  }

  // Initialize LCD
  lcd_->loadAndInitialize(lcd_params_);

  // Initialize submap atlas
  SubmapAtlas::Parameters submap_params;
  ros::param::get("~max_submap_size", submap_params.max_submap_size);
  ros::param::get("~max_submap_distance", submap_params.max_submap_distance);
  submap_atlas_.reset(new SubmapAtlas(submap_params));

  // Subscriber
  std::string topic = "/" + robot_names_[my_id_] + "/kimera_vio_ros/pose_graph_incremental";
  local_pg_sub_ = nh_.subscribe(topic, 1000, &DistributedLoopClosure::localPoseGraphCallback, this);
  for (size_t id = 0; id < num_robots_; ++id) {
    if (id < my_id_) {
      std::string req_topic =
          "/" + robot_names_[id] + "/kimera_distributed/vlc_requests";
      ros::Subscriber req_sub = nh_.subscribe(
          req_topic, 1, &DistributedLoopClosure::vlcRequestsCallback, this);
      vlc_requests_sub_.push_back(req_sub);
    }

    if (id >= my_id_) {
      std::string bow_topic =
          "/" + robot_names_[id] + "/kimera_vio_ros/bow_query";
      ros::Subscriber bow_sub = nh_.subscribe(
          bow_topic, 1000, &DistributedLoopClosure::bowCallback, this);
      bow_sub_.push_back(bow_sub);
    }

    if (id > my_id_) {
      std::string resp_topic =
          "/" + robot_names_[id] + "/kimera_distributed/vlc_responses";
      ros::Subscriber resp_sub = nh_.subscribe(
          resp_topic, 10, &DistributedLoopClosure::vlcResponsesCallback, this);
      vlc_responses_sub_.push_back(resp_sub);
    }
  }

  // Publisher
  std::string loop_closure_topic =
      "/" + robot_names_[my_id_] + "/kimera_distributed/loop_closure";
  loop_closure_pub_ = nh_.advertise<pose_graph_tools::PoseGraphEdge>(
      loop_closure_topic, 1000, false);

  std::string pose_graph_topic =
      "/" + robot_names_[my_id_] + "/kimera_distributed/pose_graph_incremental";
  pose_graph_pub_ =
      nh_.advertise<pose_graph_tools::PoseGraph>(pose_graph_topic, 1000, false);

  std::string resp_topic =
      "/" + robot_names_[my_id_] + "/kimera_distributed/vlc_responses";
  vlc_responses_pub_ =
      nh_.advertise<pose_graph_tools::VLCFrames>(resp_topic, 10, true);

  std::string req_topic =
      "/" + robot_names_[my_id_] + "/kimera_distributed/vlc_requests";
  vlc_requests_pub_ =
      nh_.advertise<pose_graph_tools::VLCRequests>(req_topic, 10, true);

  // ROS service
  pose_graph_request_server_ = nh_.advertiseService(
      "request_pose_graph", &DistributedLoopClosure::requestPoseGraphCallback, this);

  ROS_INFO_STREAM(
      "Distributed Kimera node initialized (ID = "
      << my_id_ << "). \n"
      << "Parameters: \n"
      << "alpha = " << lcd_params_.alpha_ << "\n"
      << "dist_local = " << lcd_params_.dist_local_ << "\n"
      << "max_db_results = " << lcd_params_.max_db_results_ << "\n"
      << "min_nss_factor = " << lcd_params_.min_nss_factor_ << "\n"
      << "lowe_ratio = " << lcd_params_.lowe_ratio_ << "\n"
      << "max_nrFrames_between_queries = "
      << lcd_params_.lcd_tp_params_.max_nrFrames_between_queries_ << "\n"
      << "max_nrFrames_between_islands = "
      << lcd_params_.lcd_tp_params_.max_nrFrames_between_islands_ << "\n"
      << "max_intraisland_gap = "
      << lcd_params_.lcd_tp_params_.max_intraisland_gap_ << "\n"
      << "min_matches_per_island = "
      << lcd_params_.lcd_tp_params_.min_matches_per_island_ << "\n"
      << "min_temporal_matches = "
      << lcd_params_.lcd_tp_params_.min_temporal_matches_ << "\n"
      << "max_ransac_iterations = " << lcd_params_.max_ransac_iterations_
      << "\n"
      << "mono ransac threshold = " << lcd_params_.ransac_threshold_mono_
      << "\n"
      << "mono ransac max iterations = "
      << lcd_params_.max_ransac_iterations_mono_ << "\n"
      << "mono ransac min inlier percentage = "
      << lcd_params_.ransac_inlier_percentage_mono_ << "\n"
      << "ransac_threshold = " << lcd_params_.ransac_threshold_ << "\n"
      << "geometric_verification_min_inlier_count = "
      << lcd_params_.geometric_verification_min_inlier_count_ << "\n"
      << "geometric_verification_min_inlier_percentage = "
      << lcd_params_.geometric_verification_min_inlier_percentage_ << "\n"
      << "interrobot loop closure only = " << lcd_params_.inter_robot_only_
      << "\n"
      << "maximum batch size to request VLC frames = " << vlc_batch_size_ << "\n"
      << "maximum submap size = " << submap_params.max_submap_size << "\n"
      << "maximum submap distance = " << submap_params.max_submap_distance
      << "\n");

  if (run_offline_) {
    // Run offline. Load keyframe and loop closures from file.
    loadKeyframeFromFile(log_output_dir_ + "keyframe_poses.csv");
    loadLoopClosuresFromFile(log_output_dir_ + "loop_closures.csv");
  } else {
    // Run online. In this case initialize log files to record keyframe poses and loop closures.
    if (log_output_) {
      createLogFiles();
    }
  }

  // Start verification thread
  verification_thread_.reset(
      new std::thread(&DistributedLoopClosure::runVerification, this));
  ROS_INFO("Started distributed loop closure verification thread (ID =  %d)",
           my_id_);

  // Start comms thread
  comms_thread_.reset(new std::thread(&DistributedLoopClosure::runComms, this));
  ROS_INFO("Started distributed loop closure comms thread (ID = %d)", my_id_);
}

DistributedLoopClosure::~DistributedLoopClosure() {
  ROS_INFO("Shutting down DistributedLoopClosure process on robot %d...",
           my_id_);
  should_shutdown_ = true;

  if (verification_thread_) {
    verification_thread_->join();
    verification_thread_.reset();
  }

  if (comms_thread_) {
    comms_thread_->join();
    comms_thread_.reset();
  }
}

void DistributedLoopClosure::bowCallback(
    const pose_graph_tools::BowQueryConstPtr& msg) {
  size_t robot_id = msg->robot_id;
  assert(robot_id >= my_id_);
  size_t pose_id = msg->pose_id;
  lcd::RobotPoseId vertex_query(robot_id, pose_id);
  DBoW2::BowVector bow_vec;
  BowVectorFromMsg(msg->bow_vector, &bow_vec);

  std::vector<lcd::RobotPoseId> vertex_matches;

  {  // start lcd critical section
    std::unique_lock<std::mutex> lcd_lock(lcd_mutex_);

    // Incoming bow vector is from my trajectory
    // Detect loop closures with all robots in the database
    // (including myself if inter_robot_only is set to false)
    if (robot_id == my_id_) {
      if (lcd_->detectLoop(vertex_query, bow_vec, &vertex_matches)) {
        for (const auto& vertex_match : vertex_matches) {
          lcd::PotentialVLCEdge potential_edge(vertex_query, vertex_match);

          {  // start candidate critical section. Add to candidate for request
            std::unique_lock<std::mutex> candidate_lock(candidate_lc_mutex_);
            candidate_lc_.at(robot_id).push_back(potential_edge);
          }  // end candidate critical section
        }
      }
    }

    // Incoming bow vector is from another robot
    // Detect loop closures ONLY with my trajectory
    if (robot_id != my_id_) {
      if (lcd_->detectLoopWithRobot(
              my_id_, vertex_query, bow_vec, &vertex_matches)) {
        for (const auto& vertex_match : vertex_matches) {
          lcd::PotentialVLCEdge potential_edge(vertex_query, vertex_match);

          {
            // start candidate critical section. Add to candidate for request
            std::unique_lock<std::mutex> candidate_lock(candidate_lc_mutex_);
            candidate_lc_.at(robot_id).push_back(potential_edge);
          }  // end candidate critical section
        }
      }
    }

    // Add bow vector to database
    lcd_->addBowVector(vertex_query, bow_vec);
  }  // end lcd critical section

  // Inter-robot queries will count as communication payloads
  if (robot_id != my_id_) {
    received_bow_bytes_.push_back(computeBowQueryPayloadBytes(*msg));
  }
}

void DistributedLoopClosure::localPoseGraphCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  // Parse odometry edges and create new keyframes in the submap atlas
  std::vector<pose_graph_tools::PoseGraphEdge> local_loop_closures;
  const uint64_t ts = msg->header.stamp.toNSec();

  bool incremental_pub = true;
  if (submap_atlas_->numSubmaps() == 0 && msg->nodes.size() > 0) {
    // Create the first keyframe
    // Start submap critical section
    std::unique_lock<std::mutex> submap_lock(submap_atlas_mutex_);

    gtsam::Rot3 init_rotation(msg->nodes[0].pose.orientation.w,
                              msg->nodes[0].pose.orientation.x,
                              msg->nodes[0].pose.orientation.y,
                              msg->nodes[0].pose.orientation.z);
    gtsam::Point3 init_position(msg->nodes[0].pose.position.x,
                                msg->nodes[0].pose.position.y,
                                msg->nodes[0].pose.position.z);
    gtsam::Pose3 init_pose(init_rotation, init_position);

    submap_atlas_->createKeyframe(0, init_pose, ts);
    logKeyframePose(gtsam::Symbol(robot_id_to_prefix.at(my_id_), 0),
                    init_pose);
    incremental_pub = false;
  }

  for (const auto& pg_edge : msg->edges) {
    if (pg_edge.robot_from == my_id_ &&
        pg_edge.robot_to == my_id_ &&
        pg_edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
      int frame_src = (int)pg_edge.key_from;
      int frame_dst = (int) pg_edge.key_to;
      CHECK_EQ(frame_src + 1, frame_dst);
      if (submap_atlas_->hasKeyframe(frame_src) &&
          !submap_atlas_->hasKeyframe(frame_dst)) {
        // Start submap critical section
        std::unique_lock<std::mutex> submap_lock(submap_atlas_mutex_);
        // Check that the next keyframe has the expected id
        int expected_frame_id = submap_atlas_->numKeyframes();
        if (frame_dst != expected_frame_id) {
          ROS_ERROR("Received unexpected keyframe! (received %i, expected %i)", frame_dst, expected_frame_id);
        }

        // Use odometry to initialize the next keyframe
        lcd::VLCEdge keyframe_odometry;
        VLCEdgeFromMsg(pg_edge, &keyframe_odometry);
        const auto T_src_dst = keyframe_odometry.T_src_dst_;
        const auto T_odom_src = submap_atlas_->getKeyframe(frame_src)->getPoseInOdomFrame();
        const auto T_odom_dst = T_odom_src * T_src_dst;
        submap_atlas_->createKeyframe(frame_dst, T_odom_dst, ts);

        // Save keyframe pose to file
        gtsam::Symbol symbol_dst(robot_id_to_prefix.at(my_id_), frame_dst);
        logKeyframePose(symbol_dst, T_odom_dst);
      }
    } else if (pg_edge.robot_from == my_id_ &&
        pg_edge.robot_to == my_id_ &&
        pg_edge.type == pose_graph_tools::PoseGraphEdge::LOOPCLOSE) {
      local_loop_closures.push_back(pg_edge);
    }
  }

  // Parse intra-robot loop closures
  for (const auto& pg_edge : local_loop_closures) {
    // Read loop closure between the keyframes
    lcd::VLCEdge keyframe_loop_closure;
    VLCEdgeFromMsg(pg_edge, &keyframe_loop_closure);
    const auto T_f1_f2 = keyframe_loop_closure.T_src_dst_;
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(6, 1e-2);

    {
      // Start submap critical section
      std::unique_lock<std::mutex> submap_lock(submap_atlas_mutex_);
      // Convert the loop closure to between the corresponding submaps
      const auto keyframe_src =
          CHECK_NOTNULL(submap_atlas_->getKeyframe(pg_edge.key_from));
      const auto keyframe_dst =
          CHECK_NOTNULL(submap_atlas_->getKeyframe(pg_edge.key_to));
      const auto submap_src = CHECK_NOTNULL(keyframe_src->getSubmap());
      const auto submap_dst = CHECK_NOTNULL(keyframe_dst->getSubmap());
      // Skip this loop closure if two submaps are identical or consecutive
      if (std::abs(submap_src->id() - submap_dst->id()) <= 1) continue;
      const auto T_s1_f1 = keyframe_src->getPoseInSubmapFrame();
      const auto T_s2_f2 = keyframe_dst->getPoseInSubmapFrame();
      const auto T_s1_s2 = T_s1_f1 * T_f1_f2 * (T_s2_f2.inverse());
      // Convert the loop closure to between the corresponding submaps
      gtsam::Symbol from_key(robot_id_to_prefix.at(my_id_), submap_src->id());
      gtsam::Symbol to_key(robot_id_to_prefix.at(my_id_), submap_dst->id());
      submap_loop_closures_.add(
          gtsam::BetweenFactor<gtsam::Pose3>(from_key, to_key, T_s1_s2, noise));
    }
  }

  pose_graph_tools::PoseGraph sparse_pose_graph =
      getSubmapPoseGraph(incremental_pub);
  if (sparse_pose_graph.edges.size() > 0 ||
      sparse_pose_graph.nodes.size() > 0) {
    pose_graph_pub_.publish(sparse_pose_graph);
  }
}

void DistributedLoopClosure::runVerification() {
  ros::WallRate r(1);
  while (ros::ok() && !should_shutdown_) {
    if (queued_lc_.size() == 0) {
      r.sleep();
    } else {
      verifyLoopCallback();
    }
  }
}

void DistributedLoopClosure::runComms() {
  while (ros::ok() && !should_shutdown_) {
    size_t total_candidates = updateCandidateList();
    if (total_candidates > 0) {
      requestFrames();
    }
    ros::Duration(1.0).sleep();
  }
}

void DistributedLoopClosure::requestFrames() {
  std::unordered_map<size_t, lcd::RobotPoseIdSet> vertex_ids_map;
  for (const auto robot_queue : candidate_lc_) {
    // Form list of vertex ids that needs to be requested
    for (const auto& cand : robot_queue.second) {
      if (!lcd_->frameExists(cand.vertex_src_)) {
        const size_t& robot_id = cand.vertex_src_.first;
        if (vertex_ids_map.count(robot_id) == 0) {
          vertex_ids_map[robot_id] = lcd::RobotPoseIdSet();
        }
        if (vertex_ids_map.at(robot_id).size() >= vlc_batch_size_) {
          continue;
        }
        vertex_ids_map.at(robot_id).emplace(cand.vertex_src_);
      }
      if (!lcd_->frameExists(cand.vertex_dst_)) {
        const size_t& robot_id = cand.vertex_dst_.first;
        if (vertex_ids_map.count(robot_id) == 0) {
          vertex_ids_map[robot_id] = lcd::RobotPoseIdSet();
        }
        if (vertex_ids_map.at(robot_id).size() >= vlc_batch_size_) {
          continue;
        }
        vertex_ids_map.at(robot_id).emplace(cand.vertex_dst_);
      }
    }
    // Publish or process request for the set of VLC frames
    for (const auto& robot_set : vertex_ids_map) {
      processVLCRequests(robot_set.first, robot_set.second);
    }
  }
}

void DistributedLoopClosure::verifyLoopCallback() {
  while (queued_lc_.size() > 0) {
    // Attempt to detect a single loop closure
    lcd::PotentialVLCEdge potential_edge = queued_lc_.front();
    const auto& vertex_query = potential_edge.vertex_src_;
    const auto& vertex_match = potential_edge.vertex_dst_;

    {  // start lcd critical section
      std::unique_lock<std::mutex> lcd_lock(lcd_mutex_);
      // Both frames should already exist locally
      CHECK(lcd_->frameExists(vertex_query));
      CHECK(lcd_->frameExists(vertex_match));

      // Find correspondences between frames.
      std::vector<unsigned int> i_query, i_match;
      lcd_->computeMatchedIndices(
          vertex_query, vertex_match, &i_query, &i_match);
      assert(i_query.size() == i_match.size());

      // Geometric verificaton
      gtsam::Pose3 T_query_match;
      if (lcd_->geometricVerificationNister(
              vertex_query, vertex_match, &i_query, &i_match)) {
        if (lcd_->recoverPose(
                vertex_query, vertex_match, i_query, i_match, &T_query_match)) {
          const auto frame1 = lcd_->getVLCFrame(vertex_query);
          const auto frame2 = lcd_->getVLCFrame(vertex_match);
          // Get loop closure between keyframes (for debug purpose)
          gtsam::Symbol keyframe_from(robot_id_to_prefix.at(frame1.robot_id_), frame1.pose_id_);
          gtsam::Symbol keyframe_to(robot_id_to_prefix.at(frame2.robot_id_), frame2.pose_id_);
          static const gtsam::SharedNoiseModel& noise =
              gtsam::noiseModel::Isotropic::Variance(6, 1e-2);

          // Get loop closure between the corresponding two submaps
          const auto T_s1_f1 = frame1.T_submap_pose_;
          const auto T_s2_f2 = frame2.T_submap_pose_;
          const auto T_f1_f2 = T_query_match;
          const auto T_s1_s2 = T_s1_f1 * T_f1_f2 * (T_s2_f2.inverse());
          gtsam::Symbol submap_from(robot_id_to_prefix.at(frame1.robot_id_), frame1.submap_id_);
          gtsam::Symbol submap_to(robot_id_to_prefix.at(frame2.robot_id_), frame2.submap_id_);

          // Add this loop closure if no loop closure exists between the two submaps
          // This ensures that there is at most one loop closure between every pair of submaps
          // This is a temporary solution, and will be removed once submap frontend is implemented.
          bool add_factor = true;
          for (size_t i = 0; i < submap_loop_closures_.size(); i++) {
            // check if between factor
            if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
                submap_loop_closures_[i])) {
              // convert to between factor
              const gtsam::BetweenFactor<gtsam::Pose3>& factor = *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
                  submap_loop_closures_[i]);
              gtsam::Symbol front(factor.front());
              gtsam::Symbol back(factor.back());
              if (front == submap_from && back == submap_to) {
                add_factor = false;
                break;
              }
            }
          }
          if (add_factor) {
            logLoopClosure(keyframe_from, keyframe_to, T_query_match);
            keyframe_loop_closures_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                keyframe_from, keyframe_to, T_query_match, noise));
            submap_loop_closures_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                submap_from, submap_to, T_s1_s2, noise));
          }

          ROS_INFO(
              "Verified loop (%d,%d)-(%d,%d). Total submap loop closures: %i",
              vertex_query.first,
              vertex_query.second,
              vertex_match.first,
              vertex_match.second,
              submap_loop_closures_.size());
        }
      }
    }  // end lcd critical section
    queued_lc_.pop();
  }
}

pose_graph_tools::PoseGraph DistributedLoopClosure::getSubmapPoseGraph(
    bool incremental) {
  // Start submap critical section
  std::unique_lock<std::mutex> submap_lock(submap_atlas_mutex_);
  // Fill in submap-level loop closures
  pose_graph_tools::PoseGraph out_graph;

  if (submap_atlas_->numSubmaps() == 0) {
    return out_graph;
  }

  if (incremental) {
    gtsam::NonlinearFactorGraph new_loop_closures(
        submap_loop_closures_.begin() + last_get_lc_idx_,
        submap_loop_closures_.end());
    out_graph = GtsamGraphToRos(new_loop_closures, gtsam::Values());
    last_get_lc_idx_ = submap_loop_closures_.size();
  } else {
    out_graph = GtsamGraphToRos(submap_loop_closures_, gtsam::Values());
  }
  const std::string robot_name = robot_names_[my_id_];

  // Fill in submap-level odometry
  size_t start_idx = incremental ? last_get_submap_idx_ : 0;
  size_t end_idx = submap_atlas_->numSubmaps() - 1;
  for (int submap_id = start_idx; submap_id < end_idx; ++submap_id) {
    pose_graph_tools::PoseGraphEdge edge;
    const auto submap_src = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id));
    const auto submap_dst =
        CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id + 1));
    const auto T_odom_src = submap_src->getPoseInOdomFrame();
    const auto T_odom_dst = submap_dst->getPoseInOdomFrame();
    const auto T_src_dst = (T_odom_src.inverse()) * T_odom_dst;
    edge.robot_from = my_id_;
    edge.robot_to = my_id_;
    edge.key_from = submap_src->id();
    edge.key_to = submap_dst->id();
    edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
    edge.pose = GtsamPoseToRos(T_src_dst);
    edge.header.stamp.fromNSec(submap_dst->stamp());
    // TODO(yun) add frame id param
    edge.header.frame_id = frame_id_;
    out_graph.edges.push_back(edge);
  }

  // Fill in submap nodes
  start_idx = (incremental) ? start_idx + 1 : start_idx;
  for (int submap_id = start_idx; submap_id < end_idx + 1; ++submap_id) {
    pose_graph_tools::PoseGraphNode node;
    const auto submap = CHECK_NOTNULL(submap_atlas_->getSubmap(submap_id));
    node.robot_id = my_id_;
    node.key = submap->id();
    node.header.stamp.fromNSec(submap->stamp());
    node.header.frame_id = frame_id_;
    node.pose = GtsamPoseToRos(submap->getPoseInOdomFrame());
    out_graph.nodes.push_back(node);
    out_graph.header.stamp.fromNSec(submap->stamp());
    out_graph.header.frame_id = frame_id_;
  }

  if (incremental) {
    last_get_submap_idx_ = end_idx;
  }

  return out_graph;
}

bool DistributedLoopClosure::requestPoseGraphCallback(pose_graph_tools::PoseGraphQuery::Request &request,
                                                      pose_graph_tools::PoseGraphQuery::Response &response) {
  CHECK_EQ(request.robot_id, my_id_);
  response.pose_graph = getSubmapPoseGraph();

  // Log all loop closures to file
  if (log_output_) {
    logCommStat(log_output_dir_ + "lcd_log.csv");
  }

  return true;
}

void DistributedLoopClosure::processVLCRequests(
    const size_t& robot_id,
    const lcd::RobotPoseIdSet& vertex_ids) {
  if (vertex_ids.size() == 0) {
    return;
  }

  ROS_INFO("Processing %d VLC requests.", vertex_ids.size());
  if (robot_id == my_id_) {
    // Directly request from Kimera-VIO-ROS
    {  // start vlc service critical section
      std::unique_lock<std::mutex> service_lock(vlc_service_mutex_);
      if (!requestVLCFrameService(vertex_ids)) {
        ROS_ERROR("Failed to retrieve local VLC frames on robot %d.", my_id_);
      }
    }
  } else {
    publishVLCRequests(robot_id, vertex_ids);
  }
}

void DistributedLoopClosure::publishVLCRequests(
    const size_t& robot_id,
    const lcd::RobotPoseIdSet& vertex_ids) {
  assert(vertex_ids.size() < vlc_batch_size_);

  // Create requests msg
  pose_graph_tools::VLCRequests requests_msg;
  requests_msg.header.stamp = ros::Time::now();
  requests_msg.robot_id = robot_id;
  for (const auto& vertex_id : vertex_ids) {
    // Do not request frame that already exists locally
    if (lcd_->frameExists(vertex_id)) {
      continue;
    }
    // Double check robot id
    assert(robot_id == vertex_id.first);

    requests_msg.pose_ids.push_back(vertex_id.second);
  }

  vlc_requests_pub_.publish(requests_msg);
}

bool DistributedLoopClosure::requestVLCFrameService(
    const lcd::RobotPoseIdSet& vertex_ids) {
  assert(vertex_ids.size() < vlc_batch_size_);

  // Request local VLC frames
  // Populate requested pose ids in ROS service query
  pose_graph_tools::VLCFrameQuery query;
  std::string service_name =
      "/" + robot_names_[my_id_] + "/kimera_vio_ros/vlc_frame_query";
  query.request.robot_id = my_id_;

  // Populate the pose ids to request
  for (const auto& vertex_id : vertex_ids) {
    // Do not request frame that already exists locally
    if (lcd_->frameExists(vertex_id)) {
      continue;
    }
    // We can only request via service local frames
    // Frames from other robots have to be requested by publisher
    assert(vertex_id.first == my_id_);
    query.request.pose_ids.push_back(vertex_id.second);
  }

  // Call ROS service
  if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
    ROS_ERROR_STREAM("ROS service " << service_name << " does not exist!");
    return false;
  }
  if (!ros::service::call(service_name, query)) {
    ROS_ERROR_STREAM("Could not query VLC frame!");
    return false;
  }

  // Parse response
  for (const auto& frame_msg : query.response.frames) {
    lcd::VLCFrame frame;
    VLCFrameFromMsg(frame_msg, &frame);
    frame.pruneInvalidKeypoints();
    assert(frame.robot_id_ == my_id_);
    lcd::RobotPoseId vertex_id(frame.robot_id_, frame.pose_id_);
    {  // start lcd critical section
      std::unique_lock<std::mutex> lcd_lock(lcd_mutex_);
      // Fill in submap information for this keyframe
      const auto keyframe = submap_atlas_->getKeyframe(frame.pose_id_);
      if (!keyframe) {
        ROS_WARN_STREAM("Received VLC frame" << frame.pose_id_ << " does not exist in submap atlas.");
        continue;
      }
      frame.submap_id_ = CHECK_NOTNULL(keyframe->getSubmap())->id();
      frame.T_submap_pose_ = keyframe->getPoseInSubmapFrame();
      lcd_->addVLCFrame(vertex_id, frame);
    }  // end lcd critical section
  }
  return true;
}

void DistributedLoopClosure::vlcResponsesCallback(
    const pose_graph_tools::VLCFramesConstPtr& msg) {
  for (const auto& frame_msg : msg->frames) {
    lcd::VLCFrame frame;
    VLCFrameFromMsg(frame_msg, &frame);
    lcd::RobotPoseId vertex_id(frame.robot_id_, frame.pose_id_);
    {  // start lcd critical section
      std::unique_lock<std::mutex> lcd_lock(lcd_mutex_);
      lcd_->addVLCFrame(vertex_id, frame);
    }  // end lcd critical section
    // Inter-robot request will be counted as communication
    if (frame.robot_id_ != my_id_) {
      received_vlc_bytes_.push_back(computeVLCFramePayloadBytes(frame_msg));
    }
  }
  // ROS_INFO("Received %d VLC frames. ", msg->frames.size());
}

size_t DistributedLoopClosure::updateCandidateList() {
  // return total number of candidates still missing VLC frames
  size_t total_candidates = 0;
  size_t ready_candidates = 0;
  // start candidate list critical section
  std::unique_lock<std::mutex> candidate_lock(candidate_lc_mutex_);
  for (const auto& robot_queue : candidate_lc_) {
    // Create new vector of candidates still missing VLC frames
    std::vector<lcd::PotentialVLCEdge> unresolved_candidates;
    for (const auto& candidate : robot_queue.second) {
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
  ROS_INFO("Number of loop closure candidates waiting for VLC frames: %d",
           total_candidates);
  ROS_INFO(
      "Number of loop closure candidates ready for geometric verification: %d",
      ready_candidates);
  ROS_INFO("Total submap loop closures: %d", submap_loop_closures_.size());
  return total_candidates;
}

void DistributedLoopClosure::vlcRequestsCallback(
    const pose_graph_tools::VLCRequestsConstPtr& msg) {
  if (msg->robot_id != my_id_) {
    return;
  }

  if (msg->pose_ids.size() == 0) {
    return;
  }

  // Find the vlc frames that we are missing
  lcd::RobotPoseIdSet missing_vertex_ids;
  for (const auto& pose_id : msg->pose_ids) {
    lcd::RobotPoseId vertex_id(my_id_, pose_id);
    if (!lcd_->frameExists(vertex_id)) {
      missing_vertex_ids.emplace(vertex_id);
    }
  }

  if (missing_vertex_ids.size() > 0) {  // start vlc service critical section
    std::unique_lock<std::mutex> service_lock(vlc_service_mutex_);
    if (!requestVLCFrameService(missing_vertex_ids)) {
      ROS_ERROR("Failed to retrieve local VLC frames on robot %d.", my_id_);
    }
  }

  // Publish VLC frames that was requested
  pose_graph_tools::VLCFrames requested_frames;
  requested_frames.header.stamp = ros::Time::now();
  for (const auto& pose_id : msg->pose_ids) {
    lcd::RobotPoseId vertex_id(my_id_, pose_id);
    if (lcd_->frameExists(vertex_id)) {
      pose_graph_tools::VLCFrameMsg vlc_msg;
      VLCFrameToMsg(lcd_->getVLCFrame(vertex_id), &vlc_msg);
      requested_frames.frames.push_back(vlc_msg);
    }
  }
  vlc_responses_pub_.publish(requested_frames);
}

void DistributedLoopClosure::publishLoopClosure(
    const lcd::VLCEdge& loop_closure_edge) {
  pose_graph_tools::PoseGraphEdge msg_edge;
  VLCEdgeToMsg(loop_closure_edge, &msg_edge);
  loop_closure_pub_.publish(msg_edge);
}

void DistributedLoopClosure::logCommStat(const std::string& filename) {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return;
  }
  // Header
  file << "total_verifications_mono, total_verifications, "
          "successful_verifications, total_bow_bytes, "
          "total_vlc_bytes\n";
  file << lcd_->getNumGeomVerificationsMono() << ",";
  file << lcd_->getNumGeomVerifications() << ",";
  file << keyframe_loop_closures_.size() << ",";
  file << std::accumulate(
              received_bow_bytes_.begin(), received_bow_bytes_.end(), 0)
       << ",";
  file << std::accumulate(
              received_vlc_bytes_.begin(), received_vlc_bytes_.end(), 0)
       << "\n";
  file.close();
}

void DistributedLoopClosure::createLogFiles() {
  std::string pose_file_path = log_output_dir_ + "keyframe_poses.csv";
  std::string inter_lc_file_path = log_output_dir_ + "loop_closures.csv";
  keyframe_pose_file_.open(pose_file_path);
  if (!keyframe_pose_file_.is_open())
    ROS_ERROR_STREAM("Error opening log file: " << pose_file_path);
  keyframe_pose_file_ << std::fixed << std::setprecision(15);
  keyframe_pose_file_ << "robot_index,pose_index,qx,qy,qz,qw,tx,ty,tz\n";
  keyframe_pose_file_.flush();

  loop_closure_file_.open(inter_lc_file_path);
  if (!loop_closure_file_.is_open())
    ROS_ERROR_STREAM("Error opening log file: " << inter_lc_file_path);
  loop_closure_file_ << std::fixed << std::setprecision(15);
  loop_closure_file_ << "robot1,pose1,robot2,pose2,qx,qy,qz,qw,tx,ty,tz\n";
  loop_closure_file_.flush();
}

void DistributedLoopClosure::closeLogFiles() {
  if (keyframe_pose_file_.is_open())
    keyframe_pose_file_.close();
  if (loop_closure_file_.is_open())
    loop_closure_file_.close();
}

void DistributedLoopClosure::logKeyframePose(const gtsam::Symbol &symbol_frame, const gtsam::Pose3 &T_odom_frame) {
  if (keyframe_pose_file_.is_open()) {
    gtsam::Quaternion quat = T_odom_frame.rotation().toQuaternion();
    gtsam::Point3 point = T_odom_frame.translation();
    const uint32_t robot_id = robot_prefix_to_id.at(symbol_frame.chr());
    const uint32_t frame_id = symbol_frame.index();
    keyframe_pose_file_ << robot_id << ",";
    keyframe_pose_file_ << frame_id << ",";
    keyframe_pose_file_ << quat.x() << ",";
    keyframe_pose_file_ << quat.y() << ",";
    keyframe_pose_file_ << quat.z() << ",";
    keyframe_pose_file_ << quat.w() << ",";
    keyframe_pose_file_ << point.x() << ",";
    keyframe_pose_file_ << point.y() << ",";
    keyframe_pose_file_ << point.z() << "\n";
    keyframe_pose_file_.flush();
  }
}

void DistributedLoopClosure::logLoopClosure(const gtsam::Symbol &symbol_src,
                                            const gtsam::Symbol &symbol_dst,
                                            const gtsam::Pose3 &T_src_dst) {
  if (loop_closure_file_.is_open()) {
    gtsam::Quaternion quat = T_src_dst.rotation().toQuaternion();
    gtsam::Point3 point = T_src_dst.translation();
    const uint32_t robot_src = robot_prefix_to_id.at(symbol_src.chr());
    const uint32_t frame_src = symbol_src.index();
    const uint32_t robot_dst = robot_prefix_to_id.at(symbol_dst.chr());
    const uint32_t frame_dst = symbol_dst.index();
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
    loop_closure_file_ << point.z() << "\n";
    loop_closure_file_.flush();
  }
}

void DistributedLoopClosure::loadKeyframeFromFile(const std::string &pose_file) {
  std::ifstream infile(pose_file);
  if (!infile.is_open()) {
    ROS_ERROR("Could not open specified file!");
    ros::shutdown();
  }

  size_t num_poses_read = 0;

  // Scalars that will be filled
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
    CHECK_EQ(robot_id, my_id_);
    gtsam::Pose3 T_odom_keyframe;
    T_odom_keyframe = gtsam::Pose3(gtsam::Rot3(qw,qx,qy,qz), gtsam::Point3(tx,ty,tz));
    if (submap_atlas_->hasKeyframe(frame_id))
      continue;
    if (frame_id != submap_atlas_->numKeyframes()) {
      ROS_ERROR_STREAM("Received out of ordered keyframe. Expected id:"
                           << submap_atlas_->numKeyframes()
                           << ", received=" << frame_id);
    }
    // TODO(Yun) read and write timestamp
    submap_atlas_->createKeyframe(frame_id, T_odom_keyframe, 0);
    num_poses_read++;
  }
  infile.close();
  ROS_INFO_STREAM("Loaded " << num_poses_read << " from " << pose_file);
}

void DistributedLoopClosure::loadLoopClosuresFromFile(const std::string &lc_file) {
  if (submap_atlas_->params().max_submap_size != 1) {
    ROS_ERROR("Loading loop closure from file only supports max_submap_size=1");
    ros::shutdown();
  }
  std::ifstream infile(lc_file);
  if (!infile.is_open()) {
    ROS_ERROR("Could not open specified file!");
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

    gtsam::Symbol submap_from(robot_id_to_prefix.at(robot_from), pose_from);
    gtsam::Symbol submap_to(robot_id_to_prefix.at(robot_to), pose_to);
    gtsam::Pose3 T_f1_f2;
    T_f1_f2 = gtsam::Pose3(gtsam::Rot3(qw,qx,qy,qz), gtsam::Point3(tx,ty,tz));
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(6, 1e-2);
    submap_loop_closures_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        submap_from, submap_to, T_f1_f2, noise));
    num_measurements_read++;
  }
  infile.close();
  ROS_INFO_STREAM("Loaded " << num_measurements_read << " loop closures from " << lc_file);
}

}  // namespace kimera_distributed