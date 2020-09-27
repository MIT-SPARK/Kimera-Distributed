/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <kimera_distributed/DistributedPcm.h>

using namespace kimera_distributed;

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_distributed_pcm_node");
  ros::NodeHandle nh;

  DistributedPcm dpcm(nh);

  ros::spin();

  return 0;
}
