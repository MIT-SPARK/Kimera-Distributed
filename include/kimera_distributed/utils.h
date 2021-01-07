/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <kimera_distributed/BowQuery.h>
#include <kimera_distributed/BowVector.h>
#include <kimera_distributed/VLCFrameMsg.h>
#include <kimera_distributed/VLCFrameQuery.h>
#include <kimera_distributed/types.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <ros/console.h>
#include <map>
#include <string>

namespace kimera_distributed {
    void BowVectorToMsg(const DBoW2::BowVector& bow_vec, kimera_distributed::BowVector* msg);
    void BowVectorFromMsg(const kimera_distributed::BowVector& msg, DBoW2::BowVector* bow_vec);

    void VLCFrameToMsg(const VLCFrame& frame, VLCFrameMsg* msg);
    void VLCFrameFromMsg(const VLCFrameMsg& msg, VLCFrame* frame);
    
    void VLCEdgeToMsg(const VLCEdge& edge, pose_graph_tools::PoseGraphEdge* msg);
	void VLCEdgeFromMsg(const pose_graph_tools::PoseGraphEdge& msg, VLCEdge* edge);

    gtsam::BetweenFactor<gtsam::Pose3> VLCEdgeToGtsam(const VLCEdge& vlc_edge);
    gtsam::Pose3 RosPoseToGtsam(const geometry_msgs::Pose& transform);
    geometry_msgs::Pose GtsamPoseToRos(const gtsam::Pose3& transform);

    // Convert gtsam posegaph to PoseGraph msg
    pose_graph_tools::PoseGraph GtsamGraphToRos(
        const gtsam::NonlinearFactorGraph& factors,
        const gtsam::Values& values);

    // Convert vector of gtsam poses to path msg
    nav_msgs::Path GtsamPoseTrajectoryToPath(
        const std::vector<gtsam::Pose3>& gtsam_poses);

    // Compute the payload size in a BowQuery message 
    size_t computeBowQueryPayloadBytes(const BowQuery& msg);

    // Compute the payload size of a VLC frame
    size_t computeVLCFramePayloadBytes(const VLCFrameMsg& msg);
}  // namespace kimera_distributed
