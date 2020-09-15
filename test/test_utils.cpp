/**
 * @brief  Unit-tests utility functions in Kimera-Distributed
 * @author Yulun Tian
 */
#include <ros/ros.h>
#include "gtest/gtest.h"
#include "kimera_distributed/types.h"
#include "kimera_distributed/utils.h"

using namespace kimera_distributed;

TEST(UtilsTest, BowVector) {
  DBoW2::BowVector bow_vec; 
  kimera_distributed::BowVector msg;

  bow_vec.addWeight(1, 1.0);
  bow_vec.addWeight(1, 2.0);
  bow_vec.addWeight(2, 2.5);

  BowVectorToMsg(bow_vec, &msg);
  DBoW2::BowVector bow_vec_out;
  BowVectorFromMsg(msg, &bow_vec_out);

  ASSERT_TRUE(bow_vec == bow_vec_out);

}


TEST(UtilsTest, VLCFrameConstruction) {
  RobotID robot_id = 0;
  PoseID pose_id = 0;
  std::vector<gtsam::Vector3> keypoints;
  gtsam::Vector3 p0(0.1, 0.2, 0.3);
  gtsam::Vector3 p1(0.9, 0.8, 0.7);
  keypoints.push_back(p0);
  keypoints.push_back(p1);
  
  OrbDescriptor descriptors_mat(7,7,CV_8UC1,1);

  VLCFrame frame(robot_id, pose_id, keypoints, descriptors_mat);

  ASSERT_EQ(frame.robot_id_ , robot_id);
  ASSERT_EQ(frame.pose_id_  , pose_id);
  ASSERT_LE((frame.keypoints_[0] - p0).norm(), 1e-4);
  ASSERT_LE((frame.keypoints_[1] - p1).norm(), 1e-4);
  ASSERT_LE(cv::norm(descriptors_mat - frame.descriptors_mat_),  1e-4);
  for (size_t i = 0 ; i < descriptors_mat.size().height; ++i){
    ASSERT_LE(cv::norm(frame.descriptors_vec_[i] - descriptors_mat.row(i)) , 1e-4);
  }
}


TEST(UtilsTest, VLCFrameMessage) {
  RobotID robot_id = 0;
  PoseID pose_id = 0;
  std::vector<gtsam::Vector3> keypoints;
  gtsam::Vector3 p0(0.1, 0.2, 0.3);
  gtsam::Vector3 p1(0.9, 0.8, 0.7);
  keypoints.push_back(p0);
  keypoints.push_back(p1);

  OrbDescriptor descriptors_mat(7,7,CV_8UC1,1);

  VLCFrame frame_in(robot_id, pose_id, keypoints, descriptors_mat);
  VLCFrame frame;
  VLCFrameMsg msg;
  VLCFrameToMsg(frame_in, &msg);
  VLCFrameFromMsg(msg, &frame);

  ASSERT_EQ(frame.robot_id_ , robot_id);
  ASSERT_EQ(frame.pose_id_  , pose_id);
  ASSERT_LE((frame.keypoints_[0] - p0).norm(), 1e-4);
  ASSERT_LE((frame.keypoints_[1] - p1).norm(), 1e-4);
  ASSERT_LE(cv::norm(descriptors_mat - frame.descriptors_mat_),  1e-4);
  for (size_t i = 0 ; i < descriptors_mat.size().height; ++i){
    ASSERT_LE(cv::norm(frame.descriptors_vec_[i] - descriptors_mat.row(i)) , 1e-4);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_distributed_test_utils");
  return RUN_ALL_TESTS();
}