/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <tuple>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <DBoW2/DBoW2.h>

namespace kimera_distributed {
	typedef uint32_t RobotID;
	typedef uint32_t FrameID;
	typedef std::pair<RobotID, FrameID> PoseID;

	typedef cv::Mat OrbDescriptor;
	typedef std::vector<OrbDescriptor> OrbDescriptorVec;
	struct VLCFrame {
	  VLCFrame() {}
	  VLCFrame(const PoseID& pose_id,
	           const std::vector<gtsam::Vector3>& keypoints_3d,
	           const OrbDescriptorVec& descriptors_vec,
	           const OrbDescriptor& descriptors_mat)
	      : pose_id_(pose_id),
	        keypoints_(keypoints_3d),
	        descriptors_vec_(descriptors_vec),
	        descriptors_mat_(descriptors_mat) {}

	  PoseID pose_id_;
	  std::vector<gtsam::Vector3> keypoints_;
	  OrbDescriptorVec descriptors_vec_;
	  OrbDescriptor descriptors_mat_;
	};  // struct VLCFrame

	typedef std::map<PoseID, VLCFrame, std::less<PoseID>> VLCFrameDict;


} // end namespace