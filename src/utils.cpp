/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <cv_bridge/cv_bridge.h>
#include <kimera_distributed/prefix.h>
#include <kimera_distributed/utils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
#include <cassert>

namespace kimera_distributed {

void BowVectorToMsg(const DBoW2::BowVector& bow_vec,
                    kimera_distributed::BowVector* msg) {
  msg->word_ids.clear();
  msg->word_values.clear();
  for (auto it = bow_vec.begin(); it != bow_vec.end(); ++it) {
    msg->word_ids.push_back(it->first);
    msg->word_values.push_back(it->second);
  }
}

void BowVectorFromMsg(const kimera_distributed::BowVector& msg,
                      DBoW2::BowVector* bow_vec) {
  assert(msg.word_ids.size() == msg.word_values.size());
  bow_vec->clear();
  for (size_t i = 0; i < msg.word_ids.size(); ++i) {
    bow_vec->addWeight(msg.word_ids[i], msg.word_values[i]);
  }
}

void VLCFrameToMsg(const VLCFrame& frame,
                   kimera_distributed::VLCFrameMsg* msg) {
  msg->robot_id = frame.robot_id_;
  msg->pose_id = frame.pose_id_;

  // Convert keypoints
  PointCloud keypoints;
  for (size_t i = 0; i < frame.keypoints_.size(); ++i) {
    gtsam::Vector3 p_ = frame.keypoints_[i];
    pcl::PointXYZ p(p_(0), p_(1), p_(2));
    keypoints.push_back(p);
  }
  pcl::toROSMsg(keypoints, msg->keypoints);

  // Convert descriptors
  assert(frame.descriptors_mat_.type() ==
         0);  // check that the matrix is of type CV_8U
  cv_bridge::CvImage cv_img;
  // cv_img.header   = in_msg->header; // Yulun: need to set header explicitly?
  cv_img.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  cv_img.image = frame.descriptors_mat_;
  cv_img.toImageMsg(msg->descriptors_mat);
}

void VLCFrameFromMsg(const kimera_distributed::VLCFrameMsg& msg,
                     VLCFrame* frame) {
  frame->robot_id_ = msg.robot_id;
  frame->pose_id_ = msg.pose_id;

  // Convert keypoints
  PointCloud keypoints;
  pcl::fromROSMsg(msg.keypoints, keypoints);
  frame->keypoints_.clear();
  for (size_t i = 0; i < keypoints.size(); ++i) {
    gtsam::Vector3 p(keypoints[i].x, keypoints[i].y, keypoints[i].z);
    frame->keypoints_.push_back(p);
  }

  // Convert descriptors
  sensor_msgs::ImageConstPtr ros_image_ptr(
      new sensor_msgs::Image(msg.descriptors_mat));
  frame->descriptors_mat_ =
      cv_bridge::toCvCopy(ros_image_ptr,
                          sensor_msgs::image_encodings::TYPE_8UC1)
          ->image;
  frame->initializeDescriptorsVector();
}

gtsam::BetweenFactor<gtsam::Pose3> VLCEdgeToGtsam(const VLCEdge& vlc_edge) {
  // TODO: Currently covariance is hard coded
  uint32_t robot_src = vlc_edge.vertex_src_.first;
  uint32_t frame_src = vlc_edge.vertex_src_.second;

  uint32_t robot_dst = vlc_edge.vertex_dst_.first;
  uint32_t frame_dst = vlc_edge.vertex_dst_.second;

  // Convert to gtsam key
  gtsam::Symbol src_key(robot_id_to_prefix.at(robot_src), frame_src);
  gtsam::Symbol dst_key(robot_id_to_prefix.at(robot_dst), frame_dst);

  // Create hard coded covariance
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-2);

  // Create and return between factor
  return gtsam::BetweenFactor<gtsam::Pose3>(
      src_key, dst_key, vlc_edge.T_src_dst_, noise);
}

gtsam::Pose3 RosPoseToGtsam(const geometry_msgs::Pose& transform) {
  gtsam::Pose3 pose;
  pose = gtsam::Pose3(
      gtsam::Rot3(transform.orientation.w,
                  transform.orientation.x,
                  transform.orientation.y,
                  transform.orientation.z),
      gtsam::Point3(
          transform.position.x, transform.position.y, transform.position.z));
  return pose;
}

void VLCEdgeToMsg(const VLCEdge& edge, pose_graph_tools::PoseGraphEdge* msg) {
  // Yulun: this function currently does not assign covariance!

  msg->robot_from = edge.vertex_src_.first;
  msg->key_from = edge.vertex_src_.second;
  msg->robot_to = edge.vertex_dst_.first;
  msg->key_to = edge.vertex_dst_.second;
  msg->type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;

  gtsam::Pose3 pose = edge.T_src_dst_;
  gtsam::Quaternion quat = pose.rotation().toQuaternion();
  gtsam::Point3 position = pose.translation();

  msg->pose.orientation.x = quat.x();
  msg->pose.orientation.y = quat.y();
  msg->pose.orientation.z = quat.z();
  msg->pose.orientation.w = quat.w();

  msg->pose.position.x = position.x();
  msg->pose.position.y = position.y();
  msg->pose.position.z = position.z();
}

void VLCEdgeFromMsg(const pose_graph_tools::PoseGraphEdge& msg, VLCEdge* edge) {
  edge->vertex_src_ = std::make_pair(msg.robot_from, msg.key_from);
  edge->vertex_dst_ = std::make_pair(msg.robot_to, msg.key_to);

  gtsam::Rot3 rotation(msg.pose.orientation.w,
                       msg.pose.orientation.x,
                       msg.pose.orientation.y,
                       msg.pose.orientation.z);

  gtsam::Point3 position(
      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

  gtsam::Pose3 T_src_dst(rotation, position);
  edge->T_src_dst_ = T_src_dst;
}

// Convert gtsam posegaph to PoseGraph msg
pose_graph_tools::PoseGraph GtsamGraphToRos(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& values) {
  std::vector<pose_graph_tools::PoseGraphEdge> edges;

  // first store the factors as edges
  for (size_t i = 0; i < factors.size(); i++) {
    // check if between factor
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
            factors[i])) {
      // convert to between factor
      const gtsam::BetweenFactor<gtsam::Pose3>& factor =
          *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
              factors[i]);
      // convert between factor to PoseGraphEdge type
      pose_graph_tools::PoseGraphEdge edge;
      gtsam::Symbol front(factor.front());
      gtsam::Symbol back(factor.back());
      edge.key_from = front.index();
      edge.key_to = back.index();
      edge.robot_from = robot_prefix_to_id.at(front.chr());
      edge.robot_to = robot_prefix_to_id.at(back.chr());

      if (edge.key_to == edge.key_from + 1 &&
          edge.robot_from == edge.robot_to) {  // check if odom
        edge.type = pose_graph_tools::PoseGraphEdge::ODOM;

      } else {
        edge.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
      }
      // transforms - translation
      const gtsam::Point3& translation = factor.measured().translation();
      edge.pose.position.x = translation.x();
      edge.pose.position.y = translation.y();
      edge.pose.position.z = translation.z();
      // transforms - rotation (to quaternion)
      const gtsam::Quaternion& quaternion =
          factor.measured().rotation().toQuaternion();
      edge.pose.orientation.x = quaternion.x();
      edge.pose.orientation.y = quaternion.y();
      edge.pose.orientation.z = quaternion.z();
      edge.pose.orientation.w = quaternion.w();

      // transfer covariance
      gtsam::Matrix66 covariance =
          boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
              factor.noiseModel())
              ->covariance();
      for (size_t i = 0; i < edge.covariance.size(); i++) {
        size_t row = static_cast<size_t>(i / 6);
        size_t col = i % 6;
        edge.covariance[i] = covariance(row, col);
      }
      edges.push_back(edge);
    }
  }

  std::vector<pose_graph_tools::PoseGraphNode> nodes;
  // Then store the values as nodes
  gtsam::KeyVector key_list = values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    gtsam::Symbol node_symb(key_list[i]);
    try {
      size_t robot_id = robot_prefix_to_id.at(node_symb.chr());

      pose_graph_tools::PoseGraphNode node;
      node.key = node_symb.index();
      node.robot_id = robot_id;
      const gtsam::Pose3& value = values.at<gtsam::Pose3>(key_list[i]);
      const gtsam::Point3& translation = value.translation();
      const gtsam::Quaternion& quaternion = value.rotation().toQuaternion();

      // pose - translation
      node.pose.position.x = translation.x();
      node.pose.position.y = translation.y();
      node.pose.position.z = translation.z();
      // pose - rotation (to quaternion)
      node.pose.orientation.x = quaternion.x();
      node.pose.orientation.y = quaternion.y();
      node.pose.orientation.z = quaternion.z();
      node.pose.orientation.w = quaternion.w();
      nodes.push_back(node);
    } catch (...) {
      // ignore
    }
  }

  pose_graph_tools::PoseGraph posegraph;
  posegraph.nodes = nodes;
  posegraph.edges = edges;
  return posegraph;
}

}  // namespace kimera_distributed