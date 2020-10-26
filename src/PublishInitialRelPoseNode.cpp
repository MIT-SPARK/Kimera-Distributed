/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 * Enforces the initial poses as loop closures
 * Usage: rosrun publish_initial_rel_pose_node <number of robot>
 */

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_distributed_pcm_node");
  ros::NodeHandle nh;

  int num_robots = std::atoi(argv[1]);
  ros::Publisher initial_rel_pose_pub;
  // Due to architecture, only need to publish to robot 0
  initial_rel_pose_pub = nh.advertise<pose_graph_tools::PoseGraphEdge>(
      "/kimera0/kimera_distributed/loop_closure", num_robots);

  if (num_robots < 2) {
    ROS_ERROR("Less than 2 two robots. No relative initial poses. ");
    return 0;
  }

  ros::Rate rate(30);

  while (initial_rel_pose_pub.getNumSubscribers() == 0) rate.sleep();

  // Read the initial poses
  std::vector<gtsam::Pose3> initial_poses;
  for (size_t i = 0; i < num_robots; i++) {
    std::string robot_name = "/kimera" + std::to_string(i);
    double x_pos, y_pos, z_pos, x_quat, y_quat, z_quat, w_quat;
    if (!ros::param::get(robot_name + "/position/x", x_pos) ||
        !ros::param::get(robot_name + "/position/y", y_pos) ||
        !ros::param::get(robot_name + "/position/z", z_pos) ||
        !ros::param::get(robot_name + "/orientation/x", x_quat) ||
        !ros::param::get(robot_name + "/orientation/y", y_quat) ||
        !ros::param::get(robot_name + "/orientation/z", z_quat) ||
        !ros::param::get(robot_name + "/orientation/w", w_quat)) {
      ROS_ERROR("Unable to retrieve initial pose for robot %d", i);
      return 0;
    }
    initial_poses.push_back(
        gtsam::Pose3(gtsam::Rot3(w_quat, x_quat, y_quat, z_quat),
                     gtsam::Point3(x_pos, y_pos, z_pos)));
  }

  // Create the relative edges
  for (size_t i = 1; i < num_robots; i++) {
    gtsam::Pose3 pose0_T_posei = initial_poses[0].between(initial_poses[i]);
    pose_graph_tools::PoseGraphEdge relative_edge;
    relative_edge.robot_from = 0;
    relative_edge.key_from = 0;
    relative_edge.robot_to = i;
    relative_edge.key_to = 0;
    relative_edge.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;

    gtsam::Quaternion quat = pose0_T_posei.rotation().toQuaternion();
    gtsam::Point3 position = pose0_T_posei.translation();

    relative_edge.pose.orientation.x = quat.x();
    relative_edge.pose.orientation.y = quat.y();
    relative_edge.pose.orientation.z = quat.z();
    relative_edge.pose.orientation.w = quat.w();

    relative_edge.pose.position.x = position.x();
    relative_edge.pose.position.y = position.y();
    relative_edge.pose.position.z = position.z();

    initial_rel_pose_pub.publish(relative_edge);

    ROS_INFO(
        "Published relative initial transform between robot %d and robot %d",
        0,
        i);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
