/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <fstream>
#include <Eigen/Core>

std::string output_path;
std::string fix_topic;
std::ofstream output_file;

void fix_callback(const sensor_msgs::NavSatFixConstPtr &msg) {
  if (output_file.is_open()) {
    auto stamp_ns = msg->header.stamp.toNSec();
    int status = msg->status.status;

    // Read covariance
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> C;
    C << msg->position_covariance[0],
        msg->position_covariance[1],
        msg->position_covariance[2],
        msg->position_covariance[3],
        msg->position_covariance[4],
        msg->position_covariance[5],
        msg->position_covariance[6],
        msg->position_covariance[7],
        msg->position_covariance[8];
    double sigma_east = std::sqrt(C(0, 0));
    double sigma_north = std::sqrt(C(1, 1));
    double sigma_up = std::sqrt(C(2, 2));

    output_file << stamp_ns << ",";
    output_file << msg->latitude << ",";
    output_file << msg->longitude << ",";
    output_file << msg->altitude << ",";
    output_file << sigma_east << ",";
    output_file << sigma_north << ",";
    output_file << sigma_up << ",";
    output_file << status << "\n";
    output_file.flush();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "navsatfix_export_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.param("output_path", output_path, std::string(""));
  output_file.open(output_path);
  if (output_file.is_open()) {
    ROS_INFO("Exporting to %s.", output_path.c_str());
    output_file << std::fixed << std::setprecision(15);
    output_file << "stamp_ns,lat,long,alt,sigma_east,sigma_north,sigma_up,status\n";
    output_file.flush();
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