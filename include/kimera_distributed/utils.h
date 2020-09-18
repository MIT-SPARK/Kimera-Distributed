/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <kimera_distributed/BowQuery.h>
#include <kimera_distributed/BowVector.h>
#include <kimera_distributed/VLCFrameMsg.h>
#include <kimera_distributed/VLCFrameQuery.h>
#include <kimera_distributed/types.h>
#include <ros/console.h>
#include <map>
#include <string>

namespace kimera_distributed {
    void BowVectorToMsg(const DBoW2::BowVector& bow_vec, kimera_distributed::BowVector* msg);
    void BowVectorFromMsg(const kimera_distributed::BowVector& msg, DBoW2::BowVector* bow_vec);

    void VLCFrameToMsg(const VLCFrame& frame, kimera_distributed::VLCFrameMsg* msg);
    void VLCFrameFromMsg(const kimera_distributed::VLCFrameMsg& msg, VLCFrame* frame);

    gtsam::BetweenFactor<gtsam::Pose3> VLCEdgeToGtsam(const VLCEdge& vlc_edge);
    gtsam::Pose3 RosPoseToGtsam(const geometry_msgs::Pose& transform);

}  // namespace kimera_distributed
