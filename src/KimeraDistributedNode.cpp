/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <ros/ros.h>
#include <kimera_distributed/DistributedLoopClosure.h>
#include <kimera_distributed/KimeraDistributed.h>

using namespace kimera_distributed;

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_distributed_node");
  ros::NodeHandle nh;

  DistributedLoopClosure dlc(nh);

  ros::spin();

  return 0;
}
