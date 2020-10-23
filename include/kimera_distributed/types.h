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
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace kimera_distributed {

	typedef uint32_t RobotID;
	typedef uint32_t PoseID;
	typedef std::pair<RobotID, PoseID> VertexID;

	typedef cv::Mat OrbDescriptor;
	typedef std::vector<OrbDescriptor> OrbDescriptorVec;

	class VLCFrame {
	public:
		VLCFrame();
		VLCFrame(const RobotID& robot_id, const PoseID& pose_id, const std::vector<gtsam::Vector3>& keypoints_3d, const OrbDescriptor& descriptors_mat);
	  	RobotID robot_id_;
	  	PoseID pose_id_;
	  	std::vector<gtsam::Vector3> keypoints_;
	  	OrbDescriptorVec descriptors_vec_;
	  	OrbDescriptor descriptors_mat_;
		void initializeDescriptorsVector();
	};  // class VLCFrame

	struct VLCEdge {
	public:
		VLCEdge(){}
		VLCEdge(const VertexID& vertex_src, const VertexID& vertex_dst, const gtsam::Pose3 T_src_dst):
		vertex_src_(vertex_src),
		vertex_dst_(vertex_dst),
		T_src_dst_(T_src_dst) {}

		VertexID vertex_src_;
		VertexID vertex_dst_;
		gtsam::Pose3 T_src_dst_;
        bool operator==(const VLCEdge& other) {
          return (vertex_src_ == other.vertex_src_ &&
                  vertex_dst_ == other.vertex_dst_ &&
                  T_src_dst_.equals(other.T_src_dst_));
        }
	}; // struct VLCEdge

	typedef std::map<VertexID, VLCFrame, std::less<VertexID>> VLCFrameDict;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


} // end namespace