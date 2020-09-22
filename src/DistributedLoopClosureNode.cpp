/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <kimera_distributed/DistributedLoopClosure.h>
#include <kimera_distributed/DistributedPcm.h>
#include <kimera_distributed/types.h>
#include <ros/ros.h>

using namespace kimera_distributed;

class DistributedRobustLoopDetector {
 public:
  DistributedRobustLoopDetector(const ros::NodeHandle& n)
      : dlc_(n), dpcm_(n), nh_(n) {
    timer_ = nh_.createTimer(
        ros::Rate(1.0), &DistributedRobustLoopDetector::timerLoop, this);
  }
  ~DistributedRobustLoopDetector() {}

 private:
  ros::NodeHandle nh_;

  // Distributed loop closure class
  DistributedLoopClosure dlc_;
  // Distributed pcm class
  DistributedPcm dpcm_;

  // Pose graph publisher
  ros::Publisher pose_graph_pub_;

  // Timer to check callback
  ros::Timer timer_;

  void timerLoop(const ros::TimerEvent& t) {
    ros::Time dlc_last_callback = dlc_.getLastCallbackTime();
    std::vector<VLCEdge> loop_closures;
    dlc_.getLoopClosures(&loop_closures);
    if (loop_closures.size() > 0 && ros::Time::now() - dlc_last_callback > ros::Duration(1.0)) {
      ROS_INFO("No new messages: run PCM and publish pose graph. ");
      std::vector<VLCEdge> loop_closures;
      dlc_.getLoopClosures(&loop_closures);
      // Save the dlc loop closures for debugging purposes
      dlc_.saveLoopClosuresToFile(
          "/home/yunchang/catkin_ws/src/Kimera-Distributed/loop_closures_" +
          std::to_string(dlc_.getRobotId()) + ".csv");
      // Add to pcm to process
      dpcm_.addLoopClosures(loop_closures);
      dpcm_.saveLoopClosuresToFile(
          "/home/yunchang/catkin_ws/src/Kimera-Distributed/pcm_loop_closures_" +
          std::to_string(dlc_.getRobotId()) + ".csv");

      // Publish pose graph
      dpcm_.publishPoseGraph();

      // Shut down since at the end
      ros::shutdown();
    } else {
      ROS_INFO("Number of detected loop closures (prior to PCM): %d", loop_closures.size());
      ROS_INFO("Time since last callback: %lf", (ros::Time::now() - dlc_last_callback).toSec());
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_distributed_loop_closure_node");
  ros::NodeHandle nh;

  DistributedRobustLoopDetector drlcd(nh);

  ros::spin();

  return 0;
}
