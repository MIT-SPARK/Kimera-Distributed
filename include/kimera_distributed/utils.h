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
#include <kimera_multi_lcd/types.h>
#include <nav_msgs/Path.h>
#include <pose_graph_tools/BowQuery.h>
#include <pose_graph_tools/BowVector.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/VLCFrameMsg.h>
#include <pose_graph_tools/VLCFrameQuery.h>
#include <ros/console.h>
#include <map>
#include <string>

namespace lcd = kimera_multi_lcd;

namespace kimera_distributed {

void VLCFrameToMsg(const lcd::VLCFrame& frame,
                   pose_graph_tools::VLCFrameMsg* msg);
void VLCFrameFromMsg(const pose_graph_tools::VLCFrameMsg& msg,
                     lcd::VLCFrame* frame);

void VLCEdgeToMsg(const lcd::VLCEdge& edge,
                  pose_graph_tools::PoseGraphEdge* msg);
void VLCEdgeFromMsg(const pose_graph_tools::PoseGraphEdge& msg,
                    lcd::VLCEdge* edge);

gtsam::BetweenFactor<gtsam::Pose3> VLCEdgeToGtsam(const lcd::VLCEdge& vlc_edge);
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

/**
 * @brief Check if the input factor graph contains a Pose3 between factor with the specified src and dst vertices
 * @param nfg
 * @param symbol_src
 * @param symbol_dst
 * @return
 */
bool hasBetweenFactor(const gtsam::NonlinearFactorGraph &nfg,
                      const gtsam::Symbol &symbol_src,
                      const gtsam::Symbol &symbol_dst);

// Compute the payload size in a BowQuery message
size_t computeBowQueryPayloadBytes(const pose_graph_tools::BowQuery& msg);

// Compute the payload size of a VLC frame
size_t computeVLCFramePayloadBytes(const pose_graph_tools::VLCFrameMsg& msg);

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
}  // namespace kimera_distributed
