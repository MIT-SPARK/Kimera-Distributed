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

using RansacProblem =
    opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using Adapter = opengv::relative_pose::CentralRelativeAdapter;
using AdapterStereo = opengv::point_cloud::PointCloudAdapter;
using RansacProblemStereo =
    opengv::sac_problems::point_cloud::PointCloudSacProblem;
using BearingVectors =
    std::vector<gtsam::Vector3, Eigen::aligned_allocator<gtsam::Vector3>>;
using DMatchVec = std::vector<cv::DMatch>;

namespace kimera_distributed {

DistributedLoopClosure::DistributedLoopClosure(const ros::NodeHandle& n)
    : nh_(n), my_id_(0), num_robots_(1), use_actionlib_(false), log_output_(false),
    alpha_(0.3), dist_local_(50), max_db_results_(5), min_nss_factor_(0.05),
    max_ransac_iterations_(1000), lowe_ratio_(0.8), ransac_threshold_(0.05),
    geometric_verification_min_inlier_count_(5),
    geometric_verification_min_inlier_percentage_(0.0)
    {
  int my_id_int = -1;
  int num_robots_int = -1;
  ros::param::get("~robot_id", my_id_int);
  ros::param::get("~num_robots", num_robots_int);
  assert(my_id_int >= 0);
  assert(num_robots_int > 0);
  my_id_ = my_id_int;
  num_robots_ = num_robots_int;
  next_pose_id_ = 0;
  latest_bowvec_.resize(num_robots_);

  // Use service or actionlib for communication
  ros::param::get("~use_actionlib", use_actionlib_);
  if (use_actionlib_)
    ROS_WARN("DistributedLoopClosure: using actionlib.");

  // Used for logging
  total_geometric_verifications_ = 0;
  received_bow_bytes_.clear();
  received_vlc_bytes_.clear();

  // Initiate orb matcher
  orb_feature_matcher_ = cv::DescriptorMatcher::create(3);

  // Path to log outputs
  log_output_ = ros::param::get("~log_output_path", log_output_dir_);

  // Visual place recognition params
  ros::param::get("~alpha", alpha_);
  ros::param::get("~dist_local", dist_local_);
  ros::param::get("~max_db_results", max_db_results_);
  ros::param::get("~min_nss_factor", min_nss_factor_);

  // Geometric verification params
  ros::param::get("~ransac_threshold_mono", ransac_threshold_mono_);
  ros::param::get("~ransac_inlier_percentage_mono", ransac_inlier_percentage_mono_);
  ros::param::get("~max_ransac_iterations_mono", max_ransac_iterations_mono_);
  ros::param::get("~lowe_ratio", lowe_ratio_);
  ros::param::get("~max_ransac_iterations", max_ransac_iterations_);
  ros::param::get("~ransac_threshold", ransac_threshold_);
  ros::param::get("~geometric_verification_min_inlier_count",
                  geometric_verification_min_inlier_count_);
  ros::param::get("~geometric_verification_min_inlier_percentage",
                  geometric_verification_min_inlier_percentage_);
  ros::param::get("~detect_interrobot_only", detect_inter_robot_only_);

  // Initialize bag-of-word database
  std::string orb_vocab_path;
  ros::param::get("~vocabulary_path", orb_vocab_path);
  OrbVocabulary vocab;
  vocab.load(orb_vocab_path);
  db_BoW_ = std::unique_ptr<OrbDatabase>(new OrbDatabase(vocab));
  shared_db_BoW_ = std::unique_ptr<OrbDatabase>(new OrbDatabase(vocab));

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

  ROS_INFO_STREAM("Distributed Kimera node initialized (ID = "
                  << my_id_ << "). \n"
                  << "Parameters: \n"
                  << "alpha = " << alpha_ << "\n"
                  << "dist_local = " << dist_local_ << "\n"
                  << "max_db_results = " << max_db_results_ << "\n"
                  << "min_nss_factor = " << min_nss_factor_ << "\n"
                  << "lowe_ratio = " << lowe_ratio_ << "\n"
                  << "max_ransac_iterations = " << max_ransac_iterations_
                  << "\n"
                  << "mono ransac threshold = " << ransac_threshold_mono_ << "\n"
                  << "mono ransac max iterations = " << max_ransac_iterations_mono_ << "\n"
                  << "mono ransac min inlier percentage = " << ransac_inlier_percentage_mono_ << "\n"
                  << "ransac_threshold = " << ransac_threshold_ << "\n"
                  << "geometric_verification_min_inlier_count = "
                  << geometric_verification_min_inlier_count_ << "\n"
                  << "geometric_verification_min_inlier_percentage = "
                  << geometric_verification_min_inlier_percentage_ << "\n"
                  << "interrobot loop closure only = " 
                  << detect_inter_robot_only_);
}

DistributedLoopClosure::~DistributedLoopClosure() {}

void DistributedLoopClosure::bowCallback(
    const kimera_vio_ros::BowQueryConstPtr& msg) {
  RobotID robot_id = msg->robot_id;
  assert(robot_id >= my_id_);
  PoseID pose_id = msg->pose_id;
  VertexID vertex_query(robot_id, pose_id);
  DBoW2::BowVector bow_vec;
  BowVectorFromMsg(msg->bow_vector, &bow_vec);
  last_callback_time_ = ros::Time::now();

  VertexID vertex_match;
  // Detect loop closures with my trajectory
  if (!detect_inter_robot_only_ || robot_id != my_id_) {
    if (detectLoopInMyDB(vertex_query, bow_vec, &vertex_match)) {
      ROS_INFO_STREAM(
          "Checking loop closure between "
          << "(" << vertex_query.first << ", " << vertex_query.second << ")"
          << " and "
          << "(" << vertex_match.first << ", " << vertex_match.second << ")");
      if (requestVLCFrame(vertex_query) && requestVLCFrame(vertex_match)) {
        // Find correspondences between frames.
        std::vector<unsigned int> i_query, i_match;
        ComputeMatchedIndices(vertex_query, vertex_match, &i_query, &i_match);
        assert(i_query.size() == i_match.size());
        gtsam::Pose3 T_query_match;
        if (geometricVerificationNister(
                vertex_query, vertex_match, &i_query, &i_match)) {
          if (recoverPose(vertex_query,
                          vertex_match,
                          i_query,
                          i_match,
                          &T_query_match)) {
            VLCEdge edge(vertex_query, vertex_match, T_query_match);
            loop_closures_.push_back(edge);
            publishLoopClosure(edge);  // Publish to pcm node
          }
        }
      }
    }
  }

  // Detect loop closures with other robots' trajectories
  if (robot_id == my_id_) {
    if (detectLoopInSharedDB(vertex_query, bow_vec, &vertex_match)) {
      ROS_INFO_STREAM(
          "Checking loop closure between "
          << "(" << vertex_query.first << ", " << vertex_query.second << ")"
          << " and "
          << "(" << vertex_match.first << ", " << vertex_match.second << ")");

      if (requestVLCFrame(vertex_query) && requestVLCFrame(vertex_match)) {
        // Find correspondences between frames.
        std::vector<unsigned int> i_query, i_match;
        ComputeMatchedIndices(vertex_query, vertex_match, &i_query, &i_match);
        assert(i_query.size() == i_match.size());
        gtsam::Pose3 T_query_match;
        if (geometricVerificationNister(
                vertex_query, vertex_match, &i_query, &i_match)) {
          if (recoverPose(vertex_query,
                          vertex_match,
                          i_query,
                          i_match,
                          &T_query_match)) {
            VLCEdge edge(vertex_query, vertex_match, T_query_match);
            loop_closures_.push_back(edge);
            publishLoopClosure(edge);  // Publish to pcm node
          }
        }
      }
    }
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

  // Add Bag-of-word vector to database
  if (robot_id == my_id_) {
    assert(pose_id == next_pose_id_);
    assert(db_BoW_->add(bow_vec) == next_pose_id_);
    next_pose_id_++;
  } else {
    uint32_t db_index = shared_db_BoW_->add(bow_vec);
    shared_db_to_vertex_[db_index] = vertex_query;
  }
  latest_bowvec_[robot_id] = bow_vec;
}

bool DistributedLoopClosure::detectLoopInMyDB(
    const VertexID& vertex_query, const DBoW2::BowVector bow_vector_query,
    VertexID* vertex_match) {
  RobotID robot_query = vertex_query.first;
  double nss_factor = db_BoW_->getVocabulary()->score(
      bow_vector_query, latest_bowvec_[robot_query]);
  int max_possible_match_id = static_cast<int>(next_pose_id_) - 1;
  if (vertex_query.first == my_id_) {
    max_possible_match_id -= dist_local_;
  }
  if (nss_factor < min_nss_factor_) {
    return false;
  }
  if (max_possible_match_id < 0) max_possible_match_id = 0;

  DBoW2::QueryResults query_result;
  db_BoW_->query(bow_vector_query, query_result, max_db_results_,
                 max_possible_match_id);

  if (!query_result.empty()) {
    DBoW2::Result best_result = query_result[0];
    if (best_result.Score >= alpha_ * nss_factor) {
      *vertex_match = std::make_pair(my_id_, best_result.Id);
      return true;
    }
  }
  return false;
}

bool DistributedLoopClosure::detectLoopInSharedDB(
    const VertexID& vertex_query, const DBoW2::BowVector bow_vector_query,
    VertexID* vertex_match) {
  RobotID robot_query = vertex_query.first;
  double nss_factor = db_BoW_->getVocabulary()->score(
      bow_vector_query, latest_bowvec_[robot_query]);

  if (nss_factor < min_nss_factor_) {
    return false;
  }
  DBoW2::QueryResults query_result;
  shared_db_BoW_->query(bow_vector_query, query_result, max_db_results_);

  if (!query_result.empty()) {
    DBoW2::Result best_result = query_result[0];
    if (best_result.Score >= alpha_ * nss_factor) {
      *vertex_match = shared_db_to_vertex_[best_result.Id];
      return true;
    }
  }
  return false;
}

bool DistributedLoopClosure::requestVLCFrameService(const VertexID &vertex_id) {
  if (vlc_frames_.find(vertex_id) != vlc_frames_.end()) {
    // Return if this frame already exists locally
    return true;
  }
  RobotID robot_id = vertex_id.first;
  PoseID pose_id = vertex_id.second;
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

  VLCFrame frame;
  VLCFrameFromMsg(query.response.frame, &frame);
  assert(frame.robot_id_ == robot_id);
  assert(frame.pose_id_ == pose_id);

  vlc_frames_[vertex_id] = frame;

  // Inter-robot requests will incur communication payloads
  if (robot_id != my_id_) {
    received_vlc_bytes_.push_back(
        computeVLCFramePayloadBytes(query.response.frame));
  }

  return true;
}

bool DistributedLoopClosure::requestVLCFrameAction(const VertexID &vertex_id) {
  if (vlc_frames_.find(vertex_id) != vlc_frames_.end()) {
    // Return if this frame already exists locally
    return true;
  }
  RobotID robot_id = vertex_id.first;
  PoseID pose_id = vertex_id.second;

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
        VLCFrame frame;
        VLCFrameFromMsg(action_result->frame, &frame);
        assert(frame.robot_id_ == robot_id);
        assert(frame.pose_id_ == pose_id);
        vlc_frames_[vertex_id] = frame;
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

bool DistributedLoopClosure::requestVLCFrame(const VertexID& vertex_id) {
  if (use_actionlib_) {
    return requestVLCFrameAction(vertex_id);
  } else {
    return requestVLCFrameService(vertex_id);
  }
}

void DistributedLoopClosure::ComputeMatchedIndices(
    const VertexID& vertex_query, const VertexID& vertex_match,
    std::vector<unsigned int>* i_query,
    std::vector<unsigned int>* i_match) const {
  assert(i_query != NULL);
  assert(i_match != NULL);
  i_query->clear();
  i_match->clear();

  // Get two best matches between frame descriptors.
  std::vector<DMatchVec> matches;

  VLCFrame frame_query = vlc_frames_.find(vertex_query)->second;
  VLCFrame frame_match = vlc_frames_.find(vertex_match)->second;

  orb_feature_matcher_->knnMatch(frame_query.descriptors_mat_,
                                 frame_match.descriptors_mat_, matches, 2u);

  const size_t& n_matches = matches.size();
  for (size_t i = 0; i < n_matches; i++) {
    const DMatchVec& match = matches[i];
    if (match.size() < 2) continue;
    if (match[0].distance < lowe_ratio_ * match[1].distance) {
      i_query->push_back(match[0].queryIdx);
      i_match->push_back(match[0].trainIdx);
    }
  }
}

bool DistributedLoopClosure::geometricVerificationNister(
    const VertexID& vertex_query,
    const VertexID& vertex_match,
    std::vector<unsigned int>* inlier_query,
    std::vector<unsigned int>* inlier_match) {
  if (!requestVLCFrame(vertex_query)) return false;
  if (!requestVLCFrame(vertex_match)) return false;
  assert(NULL != inlier_query);
  assert(NULL != inlier_match);

  std::vector<unsigned int> i_query = *inlier_query;
  std::vector<unsigned int> i_match = *inlier_match;

  BearingVectors query_versors, match_versors;

  query_versors.resize(i_query.size());
  match_versors.resize(i_match.size());
  for (size_t i = 0; i < i_match.size(); i++) {
    gtsam::Vector3 query_keypt_i =
        vlc_frames_[vertex_query].keypoints_.at(i_query[i]);
    gtsam::Vector3 match_keypt_i =
        vlc_frames_[vertex_match].keypoints_.at(i_match[i]);
    query_versors[i] = query_keypt_i.normalized();
    match_versors[i] = match_keypt_i.normalized();
  }

  Adapter adapter(match_versors, query_versors);

  // Use RANSAC to solve the central-relative-pose problem.
  opengv::sac::Ransac<RansacProblem> ransac;

  ransac.sac_model_ = std::make_shared<RansacProblem>(
      adapter, RansacProblem::Algorithm::NISTER, true);
  ransac.max_iterations_ = max_ransac_iterations_mono_;
  ransac.threshold_ = ransac_threshold_mono_;

  // Compute transformation via RANSAC.
  bool ransac_success = ransac.computeModel();

  if (ransac_success) {
    double inlier_percentage =
        static_cast<double>(ransac.inliers_.size()) / query_versors.size();

    if (inlier_percentage >= ransac_inlier_percentage_mono_) {
      if (ransac.iterations_ < max_ransac_iterations_mono_) {
        inlier_query->clear();
        inlier_match->clear();
        for (auto idx : ransac.inliers_) {
          inlier_query->push_back(i_query[idx]);
          inlier_match->push_back(i_match[idx]);
        }
        return true;
      }
    }
  }
  return false;
}

bool DistributedLoopClosure::recoverPose(
    const VertexID& vertex_query,
    const VertexID& vertex_match,
    const std::vector<unsigned int>& i_query,
    const std::vector<unsigned int>& i_match,
    gtsam::Pose3* T_query_match) {

  total_geometric_verifications_++;

  opengv::points_t f_match, f_query;
  for (size_t i = 0; i < i_match.size(); i++) {
    gtsam::Vector3 point_query =
        vlc_frames_[vertex_query].keypoints_.at(i_query[i]);
    gtsam::Vector3 point_match =
        vlc_frames_[vertex_match].keypoints_.at(i_match[i]);
    if (point_query.norm() > 1e-3 && point_match.norm() > 1e-3) {
      f_query.push_back(point_query);
      f_match.push_back(point_match);
    }
  }

  if (f_query.size() < 3) {
    ROS_INFO("Less than 3 putative correspondences.");
    return false;
  }

  AdapterStereo adapter(f_query, f_match);

  // Compute transform using RANSAC 3-point method (Arun).
  std::shared_ptr<RansacProblemStereo> ptcloudproblem_ptr(
      new RansacProblemStereo(adapter, true));
  opengv::sac::Ransac<RansacProblemStereo> ransac;
  ransac.sac_model_ = ptcloudproblem_ptr;
  ransac.max_iterations_ = max_ransac_iterations_;
  ransac.threshold_ = ransac_threshold_;

  // Compute transformation via RANSAC.
  bool ransac_success = ransac.computeModel();

  if (ransac_success) {
    if (ransac.inliers_.size() < geometric_verification_min_inlier_count_) {
      ROS_INFO_STREAM("Number of inlier correspondences after RANSAC "
                      << ransac.inliers_.size() << " is too low.");
      return false;
    }

    double inlier_percentage =
        static_cast<double>(ransac.inliers_.size()) / f_match.size();
    if (inlier_percentage < geometric_verification_min_inlier_percentage_) {
      ROS_INFO_STREAM("Percentage of inlier correspondences after RANSAC "
                      << inlier_percentage << " is too low.");
      return false;
    }

    opengv::transformation_t T = ransac.model_coefficients_;

    gtsam::Point3 estimated_translation(T(0, 3), T(1, 3), T(2, 3));
    if (ransac.inliers_.size() < 500 && estimated_translation.norm() < 1e-5) {
      ROS_WARN("Detected loop closure close to identity! ");
    }

    // Yulun: this is the relative pose from the query frame to the match frame?
    *T_query_match = gtsam::Pose3(gtsam::Rot3(T.block<3, 3>(0, 0)),
                                  gtsam::Point3(T(0, 3), T(1, 3), T(2, 3)));

    inlier_count_.push_back(ransac.inliers_.size());
    inlier_percentage_.push_back(inlier_percentage);

    ROS_INFO_STREAM("Verified loop closure!");

    return true;
  }

  return false;
}

void DistributedLoopClosure::getLoopClosures(
    std::vector<VLCEdge>* loop_closures) {
  *loop_closures = loop_closures_;
}

void DistributedLoopClosure::saveLoopClosuresToFile(
    const std::string filename) {
  std::ofstream file;
  file.open(filename);

  assert(loop_closures_.size() == inlier_count_.size());
  assert(loop_closures_.size() == inlier_percentage_.size());

  std::vector<VLCEdge> loop_closures;
  getLoopClosures(&loop_closures);

  // file format
  file << "robot1,pose1,robot2,pose2,qx,qy,qz,qw,tx,ty,tz,inlier_num,inlier_"
          "percent\n";

  for (size_t i = 0; i < loop_closures.size(); ++i) {
    VLCEdge edge = loop_closures[i];
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
    file << point.z() << ",";
    file << inlier_count_[i] << ",";
    file << inlier_percentage_[i] << "\n";
  }

  file.close();
}

void DistributedLoopClosure::publishLoopClosure(
    const VLCEdge& loop_closure_edge) {
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
  file << "total_verifications, successful_verifications, total_bow_bytes, "
          "total_vlc_bytes\n";
  file << total_geometric_verifications_ << ",";
  file << loop_closures_.size() << ",";
  file << std::accumulate(received_bow_bytes_.begin(),
                          received_bow_bytes_.end(), 0)
       << ",";
  file << std::accumulate(received_vlc_bytes_.begin(),
                          received_vlc_bytes_.end(), 0)
       << "\n";
  file.close();
}

}  // namespace kimera_distributed