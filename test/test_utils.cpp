/**
 * @brief  Unit-tests utility functions in Kimera-Distributed
 * @author Yulun Tian
 */
#include <ros/ros.h>
#include "gtest/gtest.h"
#include "kimera_distributed/utils.h"

TEST(UtilsTest, BowVector) {
  DBoW2::BowVector bow_vec; 
  kimera_distributed::BowVector msg;

  bow_vec.addWeight(1, 1.0);
  bow_vec.addWeight(1, 2.0);
  bow_vec.addWeight(2, 2.5);

  kimera_distributed::BowVectorToMsg(bow_vec, &msg);
  DBoW2::BowVector bow_vec_out;
  kimera_distributed::BowVectorFromMsg(msg, &bow_vec_out);

  ASSERT_TRUE(bow_vec == bow_vec_out);

}


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_distributed_test_utils");
  return RUN_ALL_TESTS();
}