/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <kimera_distributed/types.h>
#include <kimera_vio_ros/BowQuery.h>
#include <kimera_vio_ros/BowVector.h>
#include <kimera_vio_ros/VLCFrameMsg.h>
#include <kimera_vio_ros/VLCFrameQuery.h>
#include <nav_msgs/Path.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <ros/console.h>
#include <map>
#include <string>

namespace kimera_distributed {
void BowVectorToMsg(const DBoW2::BowVector& bow_vec,
                    kimera_vio_ros::BowVector* msg);
void BowVectorFromMsg(const kimera_vio_ros::BowVector& msg,
                      DBoW2::BowVector* bow_vec);

void VLCFrameToMsg(const VLCFrame& frame, kimera_vio_ros::VLCFrameMsg* msg);
void VLCFrameFromMsg(const kimera_vio_ros::VLCFrameMsg& msg, VLCFrame* frame);

void VLCEdgeToMsg(const VLCEdge& edge, pose_graph_tools::PoseGraphEdge* msg);
void VLCEdgeFromMsg(const pose_graph_tools::PoseGraphEdge& msg, VLCEdge* edge);

gtsam::BetweenFactor<gtsam::Pose3> VLCEdgeToGtsam(const VLCEdge& vlc_edge);
gtsam::Pose3 RosPoseToGtsam(const geometry_msgs::Pose& transform);
geometry_msgs::Pose GtsamPoseToRos(const gtsam::Pose3& transform);

// Convert gtsam posegaph to PoseGraph msg
pose_graph_tools::PoseGraph GtsamGraphToRos(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& values,
    const gtsam::Vector& gnc_weights = Eigen::VectorXd::Zero(0));

// Convert vector of gtsam poses to path msg
nav_msgs::Path GtsamPoseTrajectoryToPath(
    const std::vector<gtsam::Pose3>& gtsam_poses);

// Compute the payload size in a BowQuery message
size_t computeBowQueryPayloadBytes(const kimera_vio_ros::BowQuery& msg);

// Compute the payload size of a VLC frame
size_t computeVLCFramePayloadBytes(const kimera_vio_ros::VLCFrameMsg& msg);
}  // namespace kimera_distributed
