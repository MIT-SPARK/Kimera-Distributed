/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Transform.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/console.h>
#include <string>

namespace kimera_distributed {

const std::map<uint32_t, char> robot_id_to_prefix = {
    {0, 'a'},
    {1, 'b'},
    {2, 'c'},
    {3, 'd'},
    {4, 'e'},
    {5, 'f'},
    {6, 'g'},
    {7, 'h'},
    {8, 'i'},
    {9, 'j'}
};

const std::map<char, uint32_t> robot_prefix_to_id = {
    {'a', 0},
    {'b', 1},
    {'c', 2},
    {'d', 3},
    {'e', 4},
    {'f', 5},
    {'g', 6},
    {'h', 7},
    {'i', 8},
    {'j', 9}
};

gtsam::Pose3 RosPoseToGtsam(const geometry_msgs::Pose& transform);
geometry_msgs::Pose GtsamPoseToRos(const gtsam::Pose3& transform);
void GtsamPoseToRosTf(const gtsam::Pose3& pose, geometry_msgs::Transform* tf);

// Convert gtsam posegaph to PoseGraph msg
pose_graph_tools::PoseGraph GtsamGraphToRos(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& values,
    const gtsam::Vector& gnc_weights = Eigen::VectorXd::Zero(0));

// Convert vector of gtsam poses to path msg
nav_msgs::Path GtsamPoseTrajectoryToPath(const std::vector<gtsam::Pose3>& gtsam_poses);

/**
 * @brief Check if the input factor graph contains a Pose3 between factor with
 * the specified src and dst vertices
 * @param nfg
 * @param symbol_src
 * @param symbol_dst
 * @return
 */
bool hasBetweenFactor(const gtsam::NonlinearFactorGraph& nfg,
                      const gtsam::Symbol& symbol_src,
                      const gtsam::Symbol& symbol_dst);
}  // namespace kimera_distributed
