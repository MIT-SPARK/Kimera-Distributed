/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <cassert>
#include <kimera_distributed/utils.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace kimera_distributed {

void BowVectorToMsg(const DBoW2::BowVector& bow_vec, kimera_distributed::BowVector* msg) 
{
  msg->word_ids.clear();
  msg->word_values.clear();
  for (auto it = bow_vec.begin(); it != bow_vec.end(); ++it) {
    msg->word_ids.push_back(it->first);
    msg->word_values.push_back(it->second);
  }
}

void BowVectorFromMsg(const kimera_distributed::BowVector& msg, DBoW2::BowVector* bow_vec)
{
  assert(msg.word_ids.size() == msg.word_values.size());
  bow_vec->clear();
  for (size_t i = 0; i < msg.word_ids.size(); ++i){
    bow_vec->addWeight(msg.word_ids[i], msg.word_values[i]);
  }
}

void VLCFrameToMsg(const VLCFrame& frame, kimera_distributed::VLCFrameMsg* msg)
{
  msg->robot_id = frame.robot_id_;
  msg->pose_id  = frame.pose_id_;

  // Convert keypoints
  PointCloud keypoints;
  for (size_t i = 0 ; i < frame.keypoints_.size(); ++i){
    gtsam::Vector3 p_ = frame.keypoints_[i];
    pcl::PointXYZ p(p_(0), p_(1), p_(2));
    keypoints.push_back(p);
  }
  pcl::toROSMsg(keypoints, msg->keypoints);

  // Convert descriptors
  assert(frame.descriptors_mat_.type() == 0); // check that the matrix is of type CV_8U
  cv_bridge::CvImage cv_img;
  // cv_img.header   = in_msg->header; // Yulun: need to set header explicitly?
  cv_img.encoding = sensor_msgs::image_encodings::TYPE_8UC1; 
  cv_img.image    = frame.descriptors_mat_; 
  cv_img.toImageMsg(msg->descriptors_mat);

}

void VLCFrameFromMsg(const kimera_distributed::VLCFrameMsg& msg, VLCFrame* frame)
{
  frame->robot_id_ = msg.robot_id;
  frame->pose_id_  = msg.pose_id;

  // Convert keypoints
  PointCloud keypoints;
  pcl::fromROSMsg(msg.keypoints, keypoints);
  frame->keypoints_.clear();
  for (size_t i = 0; i < keypoints.size(); ++i){
    gtsam::Vector3 p(keypoints[i].x, keypoints[i].y, keypoints[i].z);
    frame->keypoints_.push_back(p);
  }

  // Convert descriptors
  sensor_msgs::ImageConstPtr ros_image_ptr(new sensor_msgs::Image( msg.descriptors_mat ));
  frame->descriptors_mat_ = cv_bridge::toCvCopy(ros_image_ptr, sensor_msgs::image_encodings::TYPE_8UC1)->image;
  frame->initializeDescriptorsVector(); 
}

}  // namespace kimera_distributed