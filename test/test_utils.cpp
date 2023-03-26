/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <nav_msgs/Path.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/console.h>
#include <string>

#include "kimera_distributed/utils.h"

namespace kimera_distributed {

TEST(UtilsTest, RosPoseToGtsam) {
  geometry_msgs::Pose ros_pose;
  ros_pose.position.x = 1.0;
  ros_pose.position.y = 2.0;
  ros_pose.position.z = 3.0;

  ros_pose.orientation.x = 0.0;
  ros_pose.orientation.y = 1.0;
  ros_pose.orientation.z = 0.0;
  ros_pose.orientation.w = 0.0;

  gtsam::Pose3 ros_pose_converted = RosPoseToGtsam(ros_pose);
  EXPECT_TRUE(gtsam::assert_equal(
      ros_pose_converted,
      gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(1, 2, 3))));
  EXPECT_EQ(ros_pose, GtsamPoseToRos(ros_pose_converted));
}

TEST(UtilsTest, GtsamGraphToRos) {
  gtsam::NonlinearFactorGraph gtsam_graph;
  gtsam::Values gtsam_values;

  for (size_t i = 0; i < 3; i++) {
    gtsam_values.insert(
        gtsam::Symbol('a', i),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0)));
  }
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-2);
  gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0),
      gtsam::Symbol('a', 1),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      noise));

  gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 1),
      gtsam::Symbol('a', 2),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      noise));

  gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 2),
      gtsam::Symbol('a', 0),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      noise));

  pose_graph_tools::PoseGraph ros_graph = GtsamGraphToRos(gtsam_graph, gtsam_values);

  EXPECT_EQ(ros_graph.edges.size(), 3);
  EXPECT_EQ(ros_graph.nodes.size(), 3);

  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(static_cast<double>(i), ros_graph.nodes[i].pose.position.x);
    EXPECT_EQ(1.0, ros_graph.edges[i].pose.position.x);
  }
}

TEST(UtilsTest, GtsamPoseTrajectoryToPath) {
  ros::Time::init();
  std::vector<gtsam::Pose3> pose_path;

  for (size_t i = 0; i < 3; i++) {
    pose_path.push_back(
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0)));
  }

  nav_msgs::Path ros_path = GtsamPoseTrajectoryToPath(pose_path);

  EXPECT_EQ(3, ros_path.poses.size());
  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(static_cast<double>(i), ros_path.poses[i].pose.position.x);
  }
}

TEST(UtilsTest, hasBetweenFactor) {
  gtsam::NonlinearFactorGraph gtsam_graph;
  gtsam::Values gtsam_values;

  for (size_t i = 0; i < 2; i++) {
    gtsam_values.insert(
        gtsam::Symbol('a', i),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0)));
  }
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-2);
  gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0),
      gtsam::Symbol('a', 1),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      noise));

  EXPECT_TRUE(
      hasBetweenFactor(gtsam_graph, gtsam::Symbol('a', 0), gtsam::Symbol('a', 1)));
  EXPECT_FALSE(
      hasBetweenFactor(gtsam_graph, gtsam::Symbol('a', 0), gtsam::Symbol('a', 2)));
}

}  // namespace kimera_distributed

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
