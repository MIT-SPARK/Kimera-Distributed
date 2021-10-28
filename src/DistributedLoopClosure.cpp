/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kimera_distributed/DistributedLoopClosure.h>
#include <kimera_vio_ros/VLCFrameListAction.h>
#include <kimera_vio_ros/VLCFrameListQuery.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <string>

namespace kimera_distributed {

DistributedLoopClosure::DistributedLoopClosure(const ros::NodeHandle& n)
    : nh_(n),
      my_id_(0),
      num_robots_(1),
      use_actionlib_(false),
      log_output_(false),
      lcd_(new lcd::LoopClosureDetector) {
  int my_id_int = -1;
  int num_robots_int = -1;
  ros::param::get("~robot_id", my_id_int);
  ros::param::get("~num_robots", num_robots_int);
  assert(my_id_int >= 0);
  assert(num_robots_int > 0);
  my_id_ = my_id_int;
  num_robots_ = num_robots_int;

  // Use service or actionlib for communication
  ros::param::get("~use_actionlib", use_actionlib_);
  if (use_actionlib_) ROS_WARN("DistributedLoopClosure: using actionlib.");

  // Used for logging
  received_bow_bytes_.clear();
  received_vlc_bytes_.clear();

  // Path to log outputs
  log_output_ = ros::param::get("~log_output_path", log_output_dir_);

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

  // Set frequency of timers
  request_sleeptime_ = 10;  
  ros::param::get("~request_sleeptime", request_sleeptime_);
  verify_sleeptime_ = 0.1;  
  ros::param::get("~verify_sleeptime", verify_sleeptime_);

  // Initialize LCD
  lcd_->loadAndInitialize(lcd_params_);

  // Subscriber
  for (size_t id = my_id_; id < num_robots_; ++id) {
    std::string topic =
        "/kimera" + std::to_string(id) + "/kimera_vio_ros/bow_query";
    ros::Subscriber sub =
        nh_.subscribe(topic, 1000, &DistributedLoopClosure::bowCallback, this);
    bow_subscribers.push_back(sub);
  }

  // Publisher
  std::string loop_closure_topic =
      "/kimera" + std::to_string(my_id_) + "/kimera_distributed/loop_closure";
  loop_closure_publisher_ = nh_.advertise<pose_graph_tools::PoseGraphEdge>(
      loop_closure_topic, 1000, false);

  // Timer
  request_timer_ = nh_.createTimer(ros::Duration(request_sleeptime_), 
                                  &DistributedLoopClosure::requestFramesCallback, 
                                  this);
  verify_timer_ = nh_.createTimer(ros::Duration(verify_sleeptime_), 
                                 &DistributedLoopClosure::verifyLoopCallback, 
                                 this);

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
      << "interrobot loop closure only = " << lcd_params_.inter_robot_only_ << "\n"
      << "maximum batch size to request VLC frames = " << vlc_batch_size_ << "\n"
      << "timer sleep to request VLC frames = " << request_sleeptime_ << "\n"
      << "timer sleep to verify potential LC = " << verify_sleeptime_ << "\n");
}

DistributedLoopClosure::~DistributedLoopClosure() {}

void DistributedLoopClosure::bowCallback(
    const kimera_vio_ros::BowQueryConstPtr& msg) {
  size_t robot_id = msg->robot_id;
  assert(robot_id >= my_id_);
  size_t pose_id = msg->pose_id;
  lcd::RobotPoseId vertex_query(robot_id, pose_id);
  DBoW2::BowVector bow_vec;
  BowVectorFromMsg(msg->bow_vector, &bow_vec);
  last_callback_time_ = ros::Time::now();

  std::vector<lcd::RobotPoseId> vertex_matches;

  // Incoming bow vector is from my trajectory
  // Detect loop closures with all robots in the database
  // (including myself if inter_robot_only is set to false)
  if (robot_id == my_id_) {
    if (lcd_->detectLoop(vertex_query, bow_vec, &vertex_matches)) {
      for (const auto& vertex_match : vertex_matches) {
        // ROS_INFO_STREAM(
        //     "Detected potential loop closure between "
        //     << "(" << vertex_query.first << ", " << vertex_query.second << ")"
        //     << " and "
        //     << "(" << vertex_match.first << ", " << vertex_match.second << ")");
        lcd::PotentialVLCEdge potential_edge(vertex_query, vertex_match);
        potential_lcs_.push_back(potential_edge);
      }
    }
  }

  // Incoming bow vector is from another robot
  // Detect loop closures ONLY with my trajectory
  if (robot_id != my_id_) {
    if (lcd_->detectLoopWithRobot(my_id_, vertex_query, bow_vec, &vertex_matches)) {
      for (const auto& vertex_match : vertex_matches) {
        // ROS_INFO_STREAM(
        //     "Detected potential loop closure between "
        //     << "(" << vertex_query.first << ", " << vertex_query.second << ")"
        //     << " and "
        //     << "(" << vertex_match.first << ", " << vertex_match.second << ")");
        lcd::PotentialVLCEdge potential_edge(vertex_query, vertex_match);
        potential_lcs_.push_back(potential_edge);
      }
    }
  }

  // Add bow vector to database 
  lcd_->addBowVector(vertex_query, bow_vec);

  // Inter-robot queries will count as communication payloads
  if (robot_id != my_id_) {
    received_bow_bytes_.push_back(computeBowQueryPayloadBytes(*msg));
  }

  // Log all loop closures to file
  if (log_output_) {
    saveLoopClosuresToFile(log_output_dir_ + "loop_closures.csv");
    logCommStat(log_output_dir_ + "lcd_log.csv");
  }
}

void DistributedLoopClosure::requestFramesCallback(const ros::TimerEvent &event) {
  // Form list of vertex ids that needs to be requested
  std::set<lcd::RobotPoseId> vertex_ids;
  for (const auto &potential_edge: potential_lcs_) {
    if (!lcd_->frameExists(potential_edge.vertex_src_)) {
      vertex_ids.emplace(potential_edge.vertex_src_);
    }
    if (!lcd_->frameExists(potential_edge.vertex_dst_)) {
      vertex_ids.emplace(potential_edge.vertex_dst_);
    }
  }

  // Request the set of VLC frames
  ROS_INFO_STREAM("Requesting " << vertex_ids.size() << " frames.");
  ros::Time request_begin = ros::Time::now();
  requestVLCFrame(vertex_ids);
  ros::Duration request_time = ros::Time::now() - request_begin;
  ROS_INFO_STREAM("Request VLC frame uses " 
                  << request_time.toSec()
                  << " seconds.");

  // Update the list of potential LCs 
  // that are ready for geometric verification
  std::vector<lcd::PotentialVLCEdge> potential_lcs_new;
  for (const auto &potential_edge: potential_lcs_) {
    if (lcd_->frameExists(potential_edge.vertex_src_) &&
        lcd_->frameExists(potential_edge.vertex_dst_)) {
      potential_lcs_ready_.push_back(potential_edge);
    } else {
      potential_lcs_new.push_back(potential_edge);
    }
  }
  potential_lcs_ = potential_lcs_new;
  ROS_INFO_STREAM("Number of potential edges that need to request frames: " 
                  << potential_lcs_.size());
  ROS_INFO_STREAM("Number of potential edges ready for geometric verification: " 
                  << potential_lcs_ready_.size());
}

void DistributedLoopClosure::verifyLoopCallback(const ros::TimerEvent &event) {
  // Do nothing if no potential loop closure is ready to be checked
  if (potential_lcs_ready_.empty())
    return;

  // Attempt to detect a single loop closure
  lcd::PotentialVLCEdge potential_edge = potential_lcs_ready_.front();
  const auto &vertex_query = potential_edge.vertex_src_;
  const auto &vertex_match = potential_edge.vertex_dst_;

  // Both frames should already exist locally
  assert(lcd_->frameExists(vertex_query) && lcd_->frameExists(vertex_match));

  // Find correspondences between frames.
  std::vector<unsigned int> i_query, i_match;
  lcd_->computeMatchedIndices(
      vertex_query, vertex_match, &i_query, &i_match);
  assert(i_query.size() == i_match.size());
  
  // Geometric verificaton
  gtsam::Pose3 T_query_match;
  if (lcd_->geometricVerificationNister(
          vertex_query, vertex_match, &i_query, &i_match)) {
    if (lcd_->recoverPose(vertex_query,
                          vertex_match,
                          i_query,
                          i_match,
                          &T_query_match)) {
      lcd::VLCEdge edge(vertex_query, vertex_match, T_query_match);
      loop_closures_.push_back(edge);
      publishLoopClosure(edge);  // Publish to pcm node
    }
  }

  // Remove this potential loop closure as it's already verified
  potential_lcs_ready_.erase(potential_lcs_ready_.begin());
}

bool DistributedLoopClosure::requestVLCFrameService(
    const lcd::RobotPoseIdSet& vertex_ids) {
  // Group requested vertex_ids based on robot
  std::unordered_map<size_t, lcd::RobotPoseIdSet> vertex_ids_map;
  for (const auto &vertex_id: vertex_ids) {
    // Do not request frame that already exists locally
    if (lcd_->frameExists(vertex_id))
      continue;
    const auto robot_id = vertex_id.first;
    // Initialize this group if needed
    if (vertex_ids_map.find(robot_id) == vertex_ids_map.end())
      vertex_ids_map.emplace(robot_id, lcd::RobotPoseIdSet());
    vertex_ids_map[robot_id].emplace(vertex_id);
  }

  // Loop over robot to send the request
  for (const auto &it: vertex_ids_map) {
    const auto &robot_id = it.first;
    const auto &vertex_ids_group = it.second;
    if (vertex_ids_group.empty())
      continue;
    std::string service_name =
      "/kimera" + std::to_string(robot_id) + "/kimera_vio_ros/vlc_frame_query";
    
    // Populate requested pose ids in ROS message 
    kimera_vio_ros::VLCFrameListQuery query;
    query.request.robot_id = robot_id;
    for (const auto &vertex_id: vertex_ids_group) {
      assert(robot_id == vertex_id.first);
      query.request.pose_ids.push_back(vertex_id.second);
      if (query.request.pose_ids.size() > vlc_batch_size_) 
        break;
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
    for (const auto& frame_msg: query.response.frames) {
      lcd::VLCFrame frame;
      VLCFrameFromMsg(frame_msg, &frame);
      assert(frame.robot_id_ == robot_id);
      lcd::RobotPoseId vertex_id(frame.robot_id_, frame.pose_id_);
      lcd_->addVLCFrame(vertex_id, frame);
      // Inter-robot request will be counted as communication
      if (robot_id != my_id_) {
        received_vlc_bytes_.push_back(
          computeVLCFramePayloadBytes(frame_msg));
      }
    }
  }

  return true;
}

bool DistributedLoopClosure::requestVLCFrameAction(
    const lcd::RobotPoseIdSet& vertex_ids) {
  // Group requested vertex_ids based on robot
  std::unordered_map<size_t, lcd::RobotPoseIdSet> vertex_ids_map;
  for (const auto &vertex_id: vertex_ids) {
    // Do not request frame that already exists locally
    if (lcd_->frameExists(vertex_id))
      continue;
    const auto robot_id = vertex_id.first;
    // Initialize this group if needed
    if (vertex_ids_map.find(robot_id) == vertex_ids_map.end())
      vertex_ids_map.emplace(robot_id, lcd::RobotPoseIdSet());
    vertex_ids_map[robot_id].emplace(vertex_id);
  }

  // Loop over robot to send the request
  for (const auto &it: vertex_ids_map) {
    const auto &robot_id = it.first;
    const auto &vertex_ids_group = it.second;
    if (vertex_ids_group.empty())
      continue;
    
    // Initialize actionlib client
    std::string action_name =
      "/kimera" + std::to_string(robot_id) + "/kimera_vio_ros/vlc_frame_action";
    actionlib::SimpleActionClient<kimera_vio_ros::VLCFrameListAction> ac(action_name,
                                                                         true);

    // Populate actionlib goal
    kimera_vio_ros::VLCFrameListGoal goal;
    goal.robot_id = robot_id;
    for (const auto &vertex_id: vertex_ids_group) {
      assert(robot_id == vertex_id.first);
      goal.pose_ids.push_back(vertex_id.second);
      if (goal.pose_ids.size() > vlc_batch_size_) break;
    }

    // Attempt to call actionlib server
    double wait_time = 0.5;
    size_t action_attempts;
    for (action_attempts = 0; action_attempts < 5; ++action_attempts) {
      ac.sendGoal(goal);
      bool finished_before_timeout = ac.waitForResult(ros::Duration(wait_time));
      if (finished_before_timeout) {
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          // Process the received frames
          const auto action_result = ac.getResult();
          for (const auto& frame_msg: action_result->frames) {
            lcd::VLCFrame frame;
            VLCFrameFromMsg(frame_msg, &frame);
            assert(frame.robot_id_ == robot_id);
            lcd::RobotPoseId vertex_id(frame.robot_id_, frame.pose_id_);
            lcd_->addVLCFrame(vertex_id, frame);
            // Inter-robot request will be counted as communication
            if (robot_id != my_id_) {
              received_vlc_bytes_.push_back(
                computeVLCFramePayloadBytes(frame_msg));
            }
          }
        }
        ROS_INFO_STREAM("Request frames from robot " << robot_id 
                        << " succeeded with " << action_attempts + 1 
                        << " attempts.");
        break;
      } else {
        wait_time += 0.5;
      }
    }
    if (action_attempts == 5) {
      ROS_ERROR("Failed to get vlc frames from robot %d", goal.robot_id);
    }
  }
  return true;
}

bool DistributedLoopClosure::requestVLCFrame(
    const lcd::RobotPoseIdSet& vertex_ids) {
  if (use_actionlib_) {
    return requestVLCFrameAction(vertex_ids);
  } else {
    return requestVLCFrameService(vertex_ids);
  }
}


void DistributedLoopClosure::publishLoopClosure(
    const lcd::VLCEdge& loop_closure_edge) {
  pose_graph_tools::PoseGraphEdge msg_edge;
  VLCEdgeToMsg(loop_closure_edge, &msg_edge);
  loop_closure_publisher_.publish(msg_edge);
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
  file << loop_closures_.size() << ",";
  file << std::accumulate(
              received_bow_bytes_.begin(), received_bow_bytes_.end(), 0)
       << ",";
  file << std::accumulate(
              received_vlc_bytes_.begin(), received_vlc_bytes_.end(), 0)
       << "\n";
  file.close();
}

void DistributedLoopClosure::saveLoopClosuresToFile(
    const std::string filename) {
  std::ofstream file;
  file.open(filename);

  std::vector<lcd::VLCEdge> loop_closures;
  getLoopClosures(&loop_closures);

  // file format
  file << "robot1,pose1,robot2,pose2,qx,qy,qz,qw,tx,ty,tz\n";

  for (size_t i = 0; i < loop_closures.size(); ++i) {
    lcd::VLCEdge edge = loop_closures[i];
    file << edge.vertex_src_.first << ",";
    file << edge.vertex_src_.second << ",";
    file << edge.vertex_dst_.first << ",";
    file << edge.vertex_dst_.second << ",";
    gtsam::Pose3 pose = edge.T_src_dst_;
    gtsam::Quaternion quat = pose.rotation().toQuaternion();
    gtsam::Point3 point = pose.translation();
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

}  // namespace kimera_distributed