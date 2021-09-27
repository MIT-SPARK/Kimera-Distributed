/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kimera_distributed/DistributedLoopClosure.h>
#include <kimera_vio_ros/VLCFrameAction.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <memory>
#include <string>

namespace kimera_distributed {

DistributedLoopClosure::DistributedLoopClosure(const ros::NodeHandle& n)
    : nh_(n),
      my_id_(0),
      num_robots_(1),
      use_actionlib_(false),
      log_output_(false) {
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
  if (use_actionlib_)
    ROS_WARN("DistributedLoopClosure: using actionlib.");

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

  // Initialize LCD
  lcd_.loadAndInitialize(lcd_params_);
  shared_lcd_.loadAndInitialize(lcd_params_);

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
      << "interrobot loop closure only = " << lcd_params_.inter_robot_only_);
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
  if (robot_id == my_id_) {
    // Detect loop closures with my trajectory against others
    // (and also my own if inter_robot_only is set to false)
    if (shared_lcd_.detectLoop(vertex_query, bow_vec, &vertex_matches)) {
      for (const auto& vertex_match : vertex_matches) {
        ROS_INFO_STREAM(
            "Checking loop closure between "
            << "(" << vertex_query.first << ", " << vertex_query.second << ")"
            << " and "
            << "(" << vertex_match.first << ", " << vertex_match.second << ")");
        if (requestVLCFrame(vertex_query, &shared_lcd_) &&
            requestVLCFrame(vertex_match, &shared_lcd_)) {
          // Find correspondences between frames.
          std::vector<unsigned int> i_query, i_match;
          shared_lcd_.computeMatchedIndices(
              vertex_query, vertex_match, &i_query, &i_match);
          assert(i_query.size() == i_match.size());
          gtsam::Pose3 T_query_match;
          if (shared_lcd_.geometricVerificationNister(
                  vertex_query, vertex_match, &i_query, &i_match)) {
            if (shared_lcd_.recoverPose(vertex_query,
                                        vertex_match,
                                        i_query,
                                        i_match,
                                        &T_query_match)) {
              lcd::VLCEdge edge(vertex_query, vertex_match, T_query_match);
              loop_closures_.push_back(edge);
              publishLoopClosure(edge);  // Publish to pcm node
            }
          }
        }
      }
    }
    if (!lcd_params_.inter_robot_only_) {
      // shared_lcd_ mostly consists of bow vectors from other trajectories
      shared_lcd_.addBowVector(vertex_query, bow_vec);
    }
    // lcd)consistes of bow vectors from my trajectory
    lcd_.addBowVector(vertex_query, bow_vec);
  }

  if (robot_id != my_id_) {
    // Detect loop closures from other trajectory against mines
    if (lcd_.detectLoop(vertex_query, bow_vec, &vertex_matches)) {
      for (const auto& vertex_match : vertex_matches) {
        ROS_INFO_STREAM(
            "Checking loop closure between "
            << "(" << vertex_query.first << ", " << vertex_query.second << ")"
            << " and "
            << "(" << vertex_match.first << ", " << vertex_match.second << ")");
        if (requestVLCFrame(vertex_query, &lcd_) &&
            requestVLCFrame(vertex_match, &lcd_)) {
          // Find correspondences between frames.
          std::vector<unsigned int> i_query, i_match;
          lcd_.computeMatchedIndices(
              vertex_query, vertex_match, &i_query, &i_match);
          assert(i_query.size() == i_match.size());
          gtsam::Pose3 T_query_match;
          if (lcd_.geometricVerificationNister(
                  vertex_query, vertex_match, &i_query, &i_match)) {
            if (lcd_.recoverPose(vertex_query,
                                 vertex_match,
                                 i_query,
                                 i_match,
                                 &T_query_match)) {
              lcd::VLCEdge edge(vertex_query, vertex_match, T_query_match);
              loop_closures_.push_back(edge);
              publishLoopClosure(edge);  // Publish to pcm node
            }
          }
        }
      }
    }
    shared_lcd_.addBowVector(vertex_query, bow_vec);
  }

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

bool DistributedLoopClosure::requestVLCFrameService(
    const lcd::RobotPoseId& vertex_id,
    lcd::LoopClosureDetector* lcd) {
  if (lcd_.frameExists(vertex_id)) {
    // Return if this frame already exists locally
    return true;
  }
  size_t robot_id = vertex_id.first;
  size_t pose_id = vertex_id.second;
  std::string service_name =
      "/kimera" + std::to_string(robot_id) + "/kimera_vio_ros/vlc_frame_query";

  kimera_vio_ros::VLCFrameQuery query;
  query.request.robot_id = robot_id;
  query.request.pose_id = pose_id;
  if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
    ROS_ERROR_STREAM("ROS service " << service_name << " does not exist!");
    return false;
  }
  if (!ros::service::call(service_name, query)) {
    ROS_ERROR_STREAM("Could not query VLC frame!");
    return false;
  }

  lcd::VLCFrame frame;
  VLCFrameFromMsg(query.response.frame, &frame);
  assert(frame.robot_id_ == robot_id);
  assert(frame.pose_id_ == pose_id);

  lcd->addVLCFrame(vertex_id, frame);

  // Inter-robot requests will incur communication payloads
  if (robot_id != my_id_) {
    received_vlc_bytes_.push_back(
        computeVLCFramePayloadBytes(query.response.frame));
  }

  return true;
}

bool DistributedLoopClosure::requestVLCFrameAction(
    const lcd::RobotPoseId& vertex_id,
    lcd::LoopClosureDetector* lcd) {
  if (lcd_.frameExists(vertex_id)) {
    // Return if this frame already exists locally
    return true;
  }
  size_t robot_id = vertex_id.first;
  size_t pose_id = vertex_id.second;

  std::string action_name = "/kimera" + std::to_string(robot_id) + "/kimera_vio_ros/vlc_frame_action";
  actionlib::SimpleActionClient<kimera_vio_ros::VLCFrameAction> ac(action_name,
                                                                   true);

  double wait_time = 0.5;
  for (size_t action_attempts = 0; action_attempts < 5; ++ action_attempts){
    ROS_INFO_STREAM("Calling action server:" <<  action_name);
    kimera_vio_ros::VLCFrameGoal goal;
    goal.robot_id = robot_id;
    goal.pose_id = pose_id;
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(wait_time));
    if (finished_before_timeout) {
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Action succeeded.");
        // Process the received frame
        const auto action_result = ac.getResult();
        lcd::VLCFrame frame;
        VLCFrameFromMsg(action_result->frame, &frame);
        assert(frame.robot_id_ == robot_id);
        assert(frame.pose_id_ == pose_id);
        lcd->addVLCFrame(vertex_id, frame);
        // Inter-robot requests will incur communication payloads
        if (robot_id != my_id_) {
          received_vlc_bytes_.push_back(
              computeVLCFramePayloadBytes(action_result->frame));
        }
        return true;
      } else {
        return false;
      }
    } else {
      ROS_WARN("Action server timeout.");
      wait_time += 0.5;
    }
  }
  // Program reaches here only if all action requests have timed out.
  return false;
}

bool DistributedLoopClosure::requestVLCFrame(const lcd::RobotPoseId& vertex_id,
                                             lcd::LoopClosureDetector* lcd) {
  if (use_actionlib_) {
    return requestVLCFrameAction(vertex_id, lcd);
  } else {
    return requestVLCFrameService(vertex_id, lcd);
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
  file << "total_verifications_mono, total_verifications, successful_verifications, total_bow_bytes, "
          "total_vlc_bytes\n";
  file << lcd_.getNumGeomVerificationsMono() << ",";
  file << lcd_.getNumGeomVerifications() << ",";
  file << loop_closures_.size() << ",";
  file << std::accumulate(received_bow_bytes_.begin(),
                          received_bow_bytes_.end(), 0)
       << ",";
  file << std::accumulate(received_vlc_bytes_.begin(),
                          received_vlc_bytes_.end(), 0)
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