/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */
#include <glog/logging.h>
#include <gtsam/linear/NoiseModel.h>
#include <kimera_distributed/utils.h>
#include <ros/console.h>
#include <cassert>

namespace kimera_distributed {

gtsam::Pose3 RosPoseToGtsam(const geometry_msgs::Pose& transform) {
  gtsam::Pose3 pose;
  pose = gtsam::Pose3(
      gtsam::Rot3(transform.orientation.w,
                  transform.orientation.x,
                  transform.orientation.y,
                  transform.orientation.z),
      gtsam::Point3(transform.position.x, transform.position.y, transform.position.z));
  return pose;
}

geometry_msgs::Pose GtsamPoseToRos(const gtsam::Pose3& transform) {
  geometry_msgs::Pose pose;

  const gtsam::Point3& position = transform.translation();
  const gtsam::Quaternion& orientation = transform.rotation().toQuaternion();

  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();

  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();

  return pose;
}

void GtsamPoseToRosTf(const gtsam::Pose3& pose, geometry_msgs::Transform* tf) {
  CHECK_NOTNULL(tf);
  tf->translation.x = pose.x();
  tf->translation.y = pose.y();
  tf->translation.z = pose.z();
  const gtsam::Quaternion& quat = pose.rotation().toQuaternion();
  tf->rotation.w = quat.w();
  tf->rotation.x = quat.x();
  tf->rotation.y = quat.y();
  tf->rotation.z = quat.z();
}

// Convert gtsam posegaph to PoseGraph msg
pose_graph_tools::PoseGraph GtsamGraphToRos(const gtsam::NonlinearFactorGraph& factors,
                                            const gtsam::Values& values,
                                            const gtsam::Vector& gnc_weights) {
  std::vector<pose_graph_tools::PoseGraphEdge> edges;
  size_t single_robot_lcs = 0;
  size_t inter_robot_lcs = 0;
  size_t single_robot_inliers = 0;
  size_t inter_robot_inliers = 0;
  // first store the factors as edges
  for (size_t i = 0; i < factors.size(); i++) {
    // check if between factor
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factors[i])) {
      // convert to between factor
      const gtsam::BetweenFactor<gtsam::Pose3>& factor =
          *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factors[i]);
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
        if (edge.robot_from == edge.robot_to) {
          single_robot_lcs++;
        } else {
          inter_robot_lcs++;
        }
        if (gnc_weights.size() == factors.size() && gnc_weights(i) < 0.5) {
          edge.type = pose_graph_tools::PoseGraphEdge::REJECTED_LOOPCLOSE;
        } else {
          if (edge.robot_from == edge.robot_to) {
            single_robot_inliers++;
          } else {
            inter_robot_inliers++;
          }
        }
      }

      edge.pose = GtsamPoseToRos(factor.measured());

      // transfer covariance
      gtsam::Matrix66 covariance =
          boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(factor.noiseModel())
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

      node.pose = GtsamPoseToRos(value);
      nodes.push_back(node);
    } catch (...) {
      // ignore
    }
  }

  pose_graph_tools::PoseGraph posegraph;
  posegraph.nodes = nodes;
  posegraph.edges = edges;
  // ROS_INFO(
  //     "Detected %d single robot loop closures with %d inliers and %d "
  //     "inter-robot loop closures with %d inliers. ",
  //     single_robot_lcs,
  //     single_robot_inliers,
  //     inter_robot_lcs,
  //     inter_robot_inliers);
  return posegraph;
}

nav_msgs::Path GtsamPoseTrajectoryToPath(const std::vector<gtsam::Pose3>& gtsam_poses) {
  nav_msgs::Path msg;
  msg.header.frame_id = "/world";
  msg.header.stamp = ros::Time::now();
  for (size_t i = 0; i < gtsam_poses.size(); i++) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = GtsamPoseToRos(gtsam_poses[i]);
    msg.poses.push_back(pose_stamped);
  }
  return msg;
}

bool hasBetweenFactor(const gtsam::NonlinearFactorGraph& nfg,
                      const gtsam::Symbol& symbol_src,
                      const gtsam::Symbol& symbol_dst) {
  for (size_t i = 0; i < nfg.size(); i++) {
    // check if between factor
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(nfg[i])) {
      // convert to between factor
      const gtsam::BetweenFactor<gtsam::Pose3>& factor =
          *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(nfg[i]);
      gtsam::Symbol front(factor.front());
      gtsam::Symbol back(factor.back());
      if (front == symbol_src && back == symbol_dst) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace kimera_distributed