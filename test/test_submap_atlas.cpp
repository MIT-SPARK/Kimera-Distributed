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

#include "kimera_distributed/SubmapAtlas.h"

namespace kimera_distributed {

SubmapAtlas createDefaultAtlas() {
  SubmapAtlas::Parameters submap_params;
  submap_params.max_submap_size = 3;
  submap_params.max_submap_distance = 1.0;
  SubmapAtlas atlas(submap_params);
  return atlas;
}

TEST(SubmapAtlasTest, SubmapAtlasParams) {
  SubmapAtlas submap_atlas = createDefaultAtlas();
  EXPECT_EQ(3, submap_atlas.params().max_submap_size);
  EXPECT_EQ(1.0, submap_atlas.params().max_submap_distance);
}

TEST(SubmapAtlasTest, CreateKeyFrame) {
  SubmapAtlas submap_atlas = createDefaultAtlas();
  gtsam::Pose3 T_odom_keyframe_0 =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0, 0));
  auto keyframe_0 = submap_atlas.createKeyframe(0, T_odom_keyframe_0, 1e+09);

  EXPECT_TRUE(submap_atlas.hasKeyframe(0));
  EXPECT_TRUE(submap_atlas.hasSubmap(0));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(),
                                  submap_atlas.getKeyframe(0)->getPoseInSubmapFrame()));
  EXPECT_TRUE(gtsam::assert_equal(T_odom_keyframe_0,
                                  submap_atlas.getKeyframe(0)->getPoseInOdomFrame()));

  gtsam::Pose3 T_odom_keyframe_1 =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.3, 0, 0));
  auto keyframe_1 = submap_atlas.createKeyframe(1, T_odom_keyframe_1, 2e+09);

  EXPECT_TRUE(submap_atlas.hasKeyframe(1));
  EXPECT_FALSE(submap_atlas.hasSubmap(1));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.2, 0, 0)),
                                  submap_atlas.getKeyframe(1)->getPoseInSubmapFrame()));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.2, 0, 0)),
                          submap_atlas.getLatestKeyframe()->getPoseInSubmapFrame()));
  EXPECT_TRUE(gtsam::assert_equal(T_odom_keyframe_1,
                                  submap_atlas.getKeyframe(1)->getPoseInOdomFrame()));
}

TEST(SubmapAtlasTest, CreateSubMapDistance) {
  SubmapAtlas submap_atlas = createDefaultAtlas();
  gtsam::Pose3 T_odom_keyframe_0 =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0, 0));
  auto keyframe_0 = submap_atlas.createKeyframe(0, T_odom_keyframe_0, 1e+09);

  gtsam::Pose3 T_odom_keyframe_1 =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.2, 0, 0));
  auto keyframe_1 = submap_atlas.createKeyframe(1, T_odom_keyframe_1, 2e+09);

  EXPECT_TRUE(submap_atlas.hasSubmap(0));
  EXPECT_FALSE(submap_atlas.hasSubmap(1));

  gtsam::Pose3 T_odom_keyframe_2 =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.3, 0, 0));
  auto keyframe_2 = submap_atlas.createKeyframe(2, T_odom_keyframe_2, 3e+09);

  EXPECT_TRUE(submap_atlas.hasSubmap(1));
  EXPECT_EQ(2, submap_atlas.numSubmaps());
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), keyframe_2->getPoseInSubmapFrame()));
  EXPECT_TRUE(gtsam::assert_equal(T_odom_keyframe_2, keyframe_2->getPoseInOdomFrame()));
}

TEST(SubmapAtlasTest, CreateSubMapNumFrames) {
  SubmapAtlas submap_atlas = createDefaultAtlas();
  gtsam::Pose3 T_odom_keyframe_0 =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0, 0));
  auto keyframe_0 = submap_atlas.createKeyframe(0, T_odom_keyframe_0, 1e+09);

  gtsam::Pose3 T_odom_keyframe_1 =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.2, 0, 0));
  auto keyframe_1 = submap_atlas.createKeyframe(1, T_odom_keyframe_1, 2e+09);

  gtsam::Pose3 T_odom_keyframe_2 =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.3, 0, 0));
  auto keyframe_2 = submap_atlas.createKeyframe(2, T_odom_keyframe_2, 3e+09);

  EXPECT_TRUE(submap_atlas.hasSubmap(0));
  EXPECT_FALSE(submap_atlas.hasSubmap(1));

  gtsam::Pose3 T_odom_keyframe_3 =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.4, 0, 0));
  auto keyframe_3 = submap_atlas.createKeyframe(3, T_odom_keyframe_3, 4e+09);

  EXPECT_TRUE(submap_atlas.hasSubmap(1));
  EXPECT_EQ(2, submap_atlas.numSubmaps());
}

}  // namespace kimera_distributed

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
