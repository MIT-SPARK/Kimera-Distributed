/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <kimera_distributed/KimeraCentralized.h>

using namespace kimera_distributed;

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_centralized_node");
  ros::NodeHandle nh;

  KimeraCentralized basestation(nh);

  ros::spin();

  return 0;
}
