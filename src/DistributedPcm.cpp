/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <gtsam/slam/BetweenFactor.h>
#include <kimera_distributed/DistributedPcm.h>
#include <kimera_distributed/prefix.h>
#include <kimera_distributed/types.h>

#include <fstream>
#include <iostream>
#include <string>
#include <cassert>

namespace kimera_distributed {

DistributedPcm::DistributedPcm(const ros::NodeHandle& n)
    : nh_(n), my_id_(-1), num_robots_(-1), b_is_frozen_(false), b_offline_mode_(false) {
  int my_id_int = -1;
  int num_robots_int = -1;
  if (!ros::param::get("~robot_id", my_id_int) ||
      !ros::param::get("~num_robots", num_robots_int)) {
    ROS_ERROR("Distributed PCM failed to get robot ID!");
    ros::shutdown();
  }
  assert(my_id_int >= 0);
  assert(num_robots_int > 0);
  my_id_ = my_id_int;
  num_robots_ = num_robots_int;

  b_request_from_robot_.assign(num_robots_, false);

  // Get optional offline flag
  ros::param::get("~offline_mode", b_offline_mode_);
  if (b_offline_mode_) {
    if(!ros::param::get("~offline_data_path", offline_data_path_)) {
      ROS_ERROR("PCM failed to get offline data path! Shutting down...");
      ros::shutdown();
    }
  }

  // Pcm parameters
  double pcm_trans_threshold, pcm_rot_threshold;
  if (!ros::param::get("~log_output_path", log_output_path_) ||
      !ros::param::get("~pcm_threshold_translation", pcm_trans_threshold) ||
      !ros::param::get("~pcm_threshold_rotation", pcm_rot_threshold)) {
    ROS_ERROR("PCM failed to get required parameters! Shutting down... ");
    ros::shutdown();
  }

  // Initialize pcm
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(pcm_trans_threshold, pcm_rot_threshold);
  pgo_params.logOutput(log_output_path_);
  pgo_params.setIncremental();
  pgo_ = std::unique_ptr<KimeraRPGO::RobustSolver>(
      new KimeraRPGO::RobustSolver(pgo_params));

  
  if (b_offline_mode_) {
    // Offline initialization
    initializeOffline();
  }
  else {
    // Start odometry edge subscribers
    for (size_t id = 0; id < num_robots_; ++id) {
      std::string topic = "/kimera" + std::to_string(id) +
                          "/kimera_vio_ros/pose_graph_incremental";
      ros::Subscriber sub =
          nh_.subscribe(topic, 100, &DistributedPcm::odometryEdgeCallback, this);
      odom_edge_subscribers_.push_back(sub);
    }

    // Start loop closure edge subscribers
    std::string loop_closure_topic =
        "/kimera" + std::to_string(my_id_) + "/kimera_distributed/loop_closure";
    loop_closure_edge_subscriber_ = nh_.subscribe(
        loop_closure_topic, 30, &DistributedPcm::loopclosureCallback, this);
  } 

  // Initialize pose graph publisher
  pose_graph_pub_ =
      nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1, false);

  // Initialize service
  shared_lc_server_ = nh_.advertiseService(
      "shared_lc_query", &DistributedPcm::shareLoopClosuresCallback, this);
  pose_graph_request_server_ = nh_.advertiseService(
      "request_pose_graph", &DistributedPcm::requestPoseGraphCallback, this);

  // Create logs
  createLogs();

  ROS_INFO("Distributed PCM node initialized. ID: %u", my_id_);
  ROS_INFO("Rotation threshold: %f", pcm_rot_threshold);
  ROS_INFO("Translation threshold: %f", pcm_trans_threshold);
  ROS_INFO("Running offline mode: %s", b_offline_mode_ ? "true" : "false");
  ROS_INFO_STREAM("Log output path: " <<  log_output_path_);
  ROS_INFO_STREAM("Offline data path: " <<  offline_data_path_);
}

DistributedPcm::~DistributedPcm() {
  closeLogs();
}

void DistributedPcm::addLoopClosures(
    const std::vector<VLCEdge>& loop_closure_edges) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  for (auto edge : loop_closure_edges) {
    new_factors.add(VLCEdgeToGtsam(edge));
  }

  pgo_->update(new_factors, new_values, false);
  // TODO: Detect if the set of inliers changed

  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();

  return;
}

void DistributedPcm::addLoopClosure(const VLCEdge& loop_closure) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  new_factors.add(VLCEdgeToGtsam(loop_closure));

  pgo_->update(new_factors, new_values, false);
  pgo_->saveData(log_output_path_);
  // TODO: Detect if the set of inliers changed

  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();

  return;
}

std::vector<VLCEdge> DistributedPcm::getInlierLoopclosures(
    const gtsam::NonlinearFactorGraph& nfg) const {
  std::vector<VLCEdge> loop_closures;
  // Extract loop closures from the last filtered pose graph
  for (auto factor : nfg) {
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
            factor)) {
      // Check not odometry factor
      if (factor->front() + 1 != factor->back()) {
        gtsam::BetweenFactor<gtsam::Pose3> lc_edge =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
                factor);

        gtsam::Symbol src_key(lc_edge.front());
        gtsam::Symbol dst_key(lc_edge.back());
        VertexID src(robot_prefix_to_id.at(src_key.chr()), src_key.index());
        VertexID dst(robot_prefix_to_id.at(dst_key.chr()), dst_key.index());
        VLCEdge vlc_edge(src, dst, lc_edge.measured());
        if (std::find(loop_closures.begin(), loop_closures.end(), vlc_edge) ==
            loop_closures.end()) {
          loop_closures.push_back(vlc_edge);
        }
      }
    }
  }

  return loop_closures;
}

void DistributedPcm::odometryEdgeCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  // New gtsam factors and values
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  // Iterate through nodes
  for (pose_graph_tools::PoseGraphNode pg_node : msg->nodes) {
    const gtsam::Pose3 estimate = RosPoseToGtsam(pg_node.pose);
    const uint32_t robot_id = pg_node.robot_id;
    const uint32_t frame_id = pg_node.key;
    gtsam::Symbol key(robot_id_to_prefix.at(robot_id), frame_id);

    if (!values_.exists(key)) new_values.insert(key, estimate);

    saveNewPosesToLog(pg_node);
  }

  // Iterate through edge
  for (pose_graph_tools::PoseGraphEdge pg_edge : msg->edges) {
    // Get edge information
    const gtsam::Pose3 measure = RosPoseToGtsam(pg_edge.pose);
    const uint32_t prev_node = pg_edge.key_from;
    const uint32_t current_node = pg_edge.key_to;
    const uint32_t robot_from = pg_edge.robot_from;
    const uint32_t robot_to = pg_edge.robot_to;
    gtsam::Symbol from_key(robot_id_to_prefix.at(robot_from), prev_node);
    gtsam::Symbol to_key(robot_id_to_prefix.at(robot_to), current_node);

    // Check if odometry edge
    if (pg_edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
      // odometry edge
      if (robot_from != robot_to || current_node != prev_node + 1) {
        ROS_ERROR(
            "Odometry edge should connect two consecutive nodes from the same "
            "robot. ");
        continue;
      }

      // Create hard coded covariance
      static const gtsam::SharedNoiseModel& noise =
          gtsam::noiseModel::Isotropic::Variance(6, 1e-2);

      // Add to pcm TODO: covariance hard coded for now
      new_factors.add(
          gtsam::BetweenFactor<gtsam::Pose3>(from_key, to_key, measure, noise));

      saveNewOdometryToLog(pg_edge);
    }
  }

  pgo_->update(new_factors, new_values, false);
  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();
}

void DistributedPcm::loopclosureCallback(
    const pose_graph_tools::PoseGraphEdge::ConstPtr& msg) {
  VLCEdge new_loop_closure;
  VLCEdgeFromMsg(*msg, &new_loop_closure);

  addLoopClosure(new_loop_closure);

  // For debugging
  std::vector<VLCEdge> loop_closures = getInlierLoopclosures(nfg_);
  saveLoopClosuresToFile(loop_closures,
                         log_output_path_ + "pcm_loop_closures.csv");
}

void DistributedPcm::saveLoopClosuresToFile(
    const std::vector<VLCEdge>& loop_closures, const std::string& filename) {
  // ROS_INFO_STREAM("Saving pcm processed loop closures to " << filename);
  std::ofstream file;
  file.open(filename);

  // file format
  file << "robot1,pose1,robot2,pose2,qx,qy,qz,qw,tx,ty,tz\n";

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
    file << point.z() << "\n";
  }

  file.close();
}

void DistributedPcm::publishPoseGraph() const {
  pose_graph_tools::PoseGraph pose_graph_msg = GtsamGraphToRos(nfg_, values_);
  pose_graph_pub_.publish(pose_graph_msg);
}

void DistributedPcm::querySharedLoopClosures(
    std::vector<pose_graph_tools::PoseGraphEdge>* shared_lc) {
  for (size_t id = 0; id < my_id_; id++) {
    std::string service_name =
        "/kimera" + std::to_string(id) + "/distributed_pcm/shared_lc_query";
    requestSharedLoopClosures query;
    query.request.robot_id = my_id_;
    if (!ros::service::call(service_name, query)) {
      ROS_ERROR("Could not query shared loop closures. ");
    }
    shared_lc->insert(shared_lc->end(), query.response.loop_closures.begin(),
                      query.response.loop_closures.end());
  }
}

bool DistributedPcm::shareLoopClosuresCallback(
    kimera_distributed::requestSharedLoopClosures::Request& request,
    kimera_distributed::requestSharedLoopClosures::Response& response) {
  auto request_robot_id = request.robot_id;
  if (request_robot_id < my_id_) {
    ROS_ERROR_STREAM("Should not receive request from robot "
                     << request_robot_id);
    return false;
  }
  if (b_request_from_robot_[request_robot_id]) {
    ROS_ERROR("Distributed PCM received repeated loop closure requests!! ");
    return false;
  }
  b_request_from_robot_[request_robot_id] = true;
  // freeze set of loop closures
  if (!b_is_frozen_) {
    loop_closures_frozen_ = getInlierLoopclosures(nfg_);
    b_is_frozen_ = true;
  }

  // ROS_INFO_STREAM("Received request from " << request_robot_id);

  for (auto lc_edge : loop_closures_frozen_) {
    if (lc_edge.vertex_src_.first == request_robot_id ||
        lc_edge.vertex_dst_.first == request_robot_id) {
      pose_graph_tools::PoseGraphEdge shared_lc_edge;
      VLCEdgeToMsg(lc_edge, &shared_lc_edge);
      response.loop_closures.push_back(shared_lc_edge);
    }
  }

  // Check if need to unfreeze loop closures
  unlockLoopClosuresIfNeeded();
  return true;
}

bool DistributedPcm::requestPoseGraphCallback(
    pose_graph_tools::PoseGraphQuery::Request& request,
    pose_graph_tools::PoseGraphQuery::Response& response) {
  // Check that id is from robot
  assert(request.robot_id == my_id_);
  if (b_request_from_robot_[request.robot_id]) {
    ROS_ERROR("Distributed PCM received repeated pose graph requests!");
    return false;
  }
  b_request_from_robot_[request.robot_id] = true;
  // freeze set of loop closures
  if (!b_is_frozen_) {
    loop_closures_frozen_ = getInlierLoopclosures(nfg_);
    b_is_frozen_ = true;
  }
  // ROS_INFO_STREAM("Received request from " << request.robot_id);

  std::vector<pose_graph_tools::PoseGraphEdge> interrobot_lc;
  querySharedLoopClosures(&interrobot_lc);

  const pose_graph_tools::PoseGraph pose_graph_msg =
      GtsamGraphToRos(nfg_, values_);

  // Filter out odometry set not from this robot
  // and add interrobot loop closures
  pose_graph_tools::PoseGraph out_graph;
  out_graph.nodes = pose_graph_msg.nodes;

  for (auto e : pose_graph_msg.edges) {
    if (e.type == pose_graph_tools::PoseGraphEdge::ODOM &&
        e.robot_from != my_id_) {
      // pass odometry edges without requested robot id
    } else {
      out_graph.edges.push_back(e);
    }
  }

  // For debugging
  std::vector<VLCEdge> output_loopclosures = loop_closures_frozen_;

  // Push the interrobot loop closures
  for (auto e : interrobot_lc) {
    out_graph.edges.push_back(e);
    VLCEdge edge;
    VLCEdgeFromMsg(e, &edge);
    output_loopclosures.push_back(edge);
  }

  response.pose_graph = out_graph;
  // Save all loop closures to file
  saveLoopClosuresToFile(
      output_loopclosures,
      log_output_path_ + "pcm_loop_closures_sent_to_dpgo.csv");

  unlockLoopClosuresIfNeeded();

  return true;
}

void DistributedPcm::unlockLoopClosuresIfNeeded() {
  bool should_unlock = true;
  for (size_t id = my_id_; id < b_request_from_robot_.size(); ++id) {
    if (!b_request_from_robot_[id]) {
      should_unlock = false;
      break;
    }
  }

  if (should_unlock) {
    b_is_frozen_ = false;
    b_request_from_robot_.assign(b_request_from_robot_.size(), false);
    // ROS_INFO("Unlock loop closures!");
  }
}


void DistributedPcm::createLogs() {
  std::string pose_file_path = log_output_path_ + "pcm_trajectory.csv";
  std::string odom_file_path = log_output_path_ + "pcm_odometry.csv";
  
  pose_file_.open(pose_file_path);
  if (!pose_file_.is_open()) ROS_ERROR_STREAM("Error opening log file: " << pose_file_path);
  pose_file_ << "robot_index,pose_index,qx,qy,qz,qw,tx,ty,tz\n";
  pose_file_.flush();

  odom_file_.open(odom_file_path);
  if (!odom_file_.is_open()) ROS_ERROR_STREAM("Error opening log file: " << odom_file_path);
  odom_file_ << "robot1,pose1,robot2,pose2,qx,qy,qz,qw,tx,ty,tz\n";
  odom_file_.flush();
}

void DistributedPcm::closeLogs() {
  pose_file_.close();
  odom_file_.close();
}

void DistributedPcm::saveNewPosesToLog(const pose_graph_tools::PoseGraphNode& node) {
  if (!pose_file_.is_open()) {
    ROS_ERROR("Pcm cannot log new pose!");
    return;
  }

  const gtsam::Pose3 estimate = RosPoseToGtsam(node.pose);
  gtsam::Quaternion quat = estimate.rotation().toQuaternion();
  gtsam::Point3 point = estimate.translation();
  const uint32_t robot_id = node.robot_id;
  const uint32_t frame_id = node.key;

  pose_file_ << robot_id << ",";
  pose_file_ << frame_id << ",";
  pose_file_ << quat.x() << ",";
  pose_file_ << quat.y() << ",";
  pose_file_ << quat.z() << ",";
  pose_file_ << quat.w() << ",";
  pose_file_ << point.x() << ",";
  pose_file_ << point.y() << ",";
  pose_file_ << point.z() << "\n";

  pose_file_.flush();

}

void DistributedPcm::saveNewOdometryToLog(const pose_graph_tools::PoseGraphEdge& edge) {

  if (!odom_file_.is_open()) {
    ROS_ERROR("Pcm cannot log new odometry!");
    return;
  }

  const gtsam::Pose3 measure = RosPoseToGtsam(edge.pose);
  gtsam::Quaternion quat = measure.rotation().toQuaternion();
  gtsam::Point3 point = measure.translation();
  const uint32_t prev_node = edge.key_from;
  const uint32_t current_node = edge.key_to;
  const uint32_t robot_from = edge.robot_from;
  const uint32_t robot_to = edge.robot_to;

  assert(robot_from == robot_to && current_node == prev_node + 1);

  odom_file_ << robot_from << ",";
  odom_file_ << prev_node << ",";
  odom_file_ << robot_to << ",";
  odom_file_ << current_node << ",";
  odom_file_ << quat.x() << ",";
  odom_file_ << quat.y() << ",";
  odom_file_ << quat.z() << ",";
  odom_file_ << quat.w() << ",";
  odom_file_ << point.x() << ",";
  odom_file_ << point.y() << ",";
  odom_file_ << point.z() << "\n";

  odom_file_.flush();
  
}

void DistributedPcm::initializeOffline() {
  gtsam::Values poses = loadPosesOffline(offline_data_path_ + "pcm_trajectory.csv");
  gtsam::NonlinearFactorGraph odometries = loadMeasurementsOffline(offline_data_path_ + "pcm_odometry.csv", true);
  gtsam::NonlinearFactorGraph loop_closures = loadMeasurementsOffline(offline_data_path_ + "loop_closures.csv", false);

  // Add odometry measurements
  pgo_->update(odometries, poses, false);

  // Add loop closures
  pgo_->update(loop_closures, gtsam::Values(), false);

  nfg_ = pgo_->getFactorsUnsafe();
  values_ = pgo_->calculateBestEstimate();
}

gtsam::Values DistributedPcm::loadPosesOffline(const std::string& filename) {
  gtsam::Values new_values;

  ROS_INFO_STREAM("Loading poses from " << filename << "...");
  std::ifstream infile(filename);
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

    // ROS_INFO("RID %i, PID %i, Quat: %f %f %f %f , Trans: %f %f %f", robot_id, frame_id, qx, qy, qz, qw, tx, ty, tz);
    
    gtsam::Pose3 estimate;
    estimate = gtsam::Pose3(gtsam::Rot3(qw,qx,qy,qz), gtsam::Point3(tx,ty,tz));
    gtsam::Symbol key(robot_id_to_prefix.at(robot_id), frame_id);
    
    if (!new_values.exists(key)) new_values.insert(key, estimate);

    num_poses_read++;
  }

  infile.close();
  ROS_INFO("Read %i poses.", num_poses_read);
  return new_values;
}

gtsam::NonlinearFactorGraph DistributedPcm::loadMeasurementsOffline(const std::string& filename, bool is_odometry) {
  gtsam::NonlinearFactorGraph new_factors;

  ROS_INFO_STREAM("Loading measurements from " << filename << "...");
  std::ifstream infile(filename);

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

    // ROS_INFO("(%i,%i)->(%i,%i), Quat: %f %f %f %f , Trans: %f %f %f", robot_from, pose_from, robot_to, pose_to, qx, qy, qz, qw, tx, ty, tz);
    if (is_odometry) {
      assert(robot_from == robot_to);
      assert(pose_from + 1 == pose_to);
    }
    
    gtsam::Pose3 measure;
    measure = gtsam::Pose3(gtsam::Rot3(qw,qx,qy,qz), gtsam::Point3(tx,ty,tz));
    gtsam::Symbol from_key(robot_id_to_prefix.at(robot_from), pose_from);
    gtsam::Symbol to_key(robot_id_to_prefix.at(robot_to), pose_to);

    // Create hard coded covariance
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(6, 1e-2);

    // Add to pcm TODO: covariance hard coded for now
    new_factors.add(
        gtsam::BetweenFactor<gtsam::Pose3>(from_key, to_key, measure, noise));

    num_measurements_read++;
  }

  infile.close();
  ROS_INFO("Read %i measurements.", num_measurements_read);
  return new_factors;
}

}  // namespace kimera_distributed