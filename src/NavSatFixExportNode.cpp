/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <fstream>

std::string output_path;
std::string fix_topic;
std::ofstream output_file;

void fix_callback(const sensor_msgs::NavSatFixConstPtr &msg) {

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "navsatfix_export_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.param("output_path", output_path, std::string(""));
  output_file.open(output_path);
  if (!output_file.is_open()) {
    ROS_INFO("Exporting to %s.", output_path.c_str());
    // TODO: header
  } else {
    ROS_ERROR("Cannot open output file: %s!", output_path.c_str());
    ros::shutdown();
  }
  pnh.param("fix_topic", fix_topic, std::string("/fix"));
  ROS_INFO("Subscribing to fix topic %s.", fix_topic.c_str());
  ros::Subscriber fix_sub = nh.subscribe(fix_topic, 100, &fix_callback);

  ros::spin();

  return 0;
}