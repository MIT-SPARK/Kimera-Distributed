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

#include "kimera_distributed/Keyframe.h"
#include "kimera_distributed/Submap.h"

namespace kimera_distributed {

TEST(SubmapTest, AddKeyframe) {
  auto keyframe_0 = std::make_shared<Keyframe>(0, 1e+09);

  Submap submap(0, 1e+09);
  submap.addKeyframe(keyframe_0);

  EXPECT_EQ(1, submap.numKeyframes());
  EXPECT_EQ(0, submap.getKeyframe(0)->id());
  EXPECT_EQ(1e+09, submap.getKeyframe(0)->stamp());
}

TEST(SubmapTest, SubmapDistancce) {
  auto keyframe_0 = std::make_shared<Keyframe>(0, 1e+09);

  Submap submap(0, 1e+09);
  submap.addKeyframe(keyframe_0);

  auto keyframe_1 = std::make_shared<Keyframe>(1, 2e+09);
  keyframe_1->setPoseInSubmapFrame(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));
  submap.addKeyframe(keyframe_1);

  EXPECT_EQ(1.0, submap.distance());
}

TEST(SubmapTest, SubmapPose) {
  Submap submap(0, 1e+09);
  gtsam::Pose3 pose_in_odom_frame(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(0, 1, 0));
  gtsam::Pose3 pose_in_world_frame(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(0, 0, 1));
  submap.setPoseInOdomFrame(pose_in_odom_frame);
  submap.setPoseInWorldFrame(pose_in_world_frame);

  EXPECT_TRUE(gtsam::assert_equal(pose_in_odom_frame, submap.getPoseInOdomFrame()));
  EXPECT_TRUE(gtsam::assert_equal(pose_in_world_frame, submap.getPoseInWorldFrame()));
}

}  // namespace kimera_distributed

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
