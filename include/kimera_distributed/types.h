/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <DBoW2/DBoW2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <opencv/cv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <tuple>

namespace kimera_distributed {

typedef uint32_t RobotID;
typedef uint32_t PoseID;
typedef std::pair<RobotID, PoseID> VertexID;

typedef cv::Mat OrbDescriptor;
typedef std::vector<OrbDescriptor> OrbDescriptorVec;

class VLCFrame {
 public:
  VLCFrame();
  VLCFrame(const RobotID& robot_id,
           const PoseID& pose_id,
           const std::vector<gtsam::Vector3>& keypoints_3d,
           const std::vector<gtsam::Vector3>& versors,
           const OrbDescriptor& descriptors_mat);
  RobotID robot_id_;
  PoseID pose_id_;
  std::vector<gtsam::Vector3> keypoints_;
  std::vector<gtsam::Vector3> versors_;
  OrbDescriptorVec descriptors_vec_;
  OrbDescriptor descriptors_mat_;
  void initializeDescriptorsVector();
};  // class VLCFrame

struct VLCEdge {
 public:
  VLCEdge() {}
  VLCEdge(const VertexID& vertex_src,
          const VertexID& vertex_dst,
          const gtsam::Pose3 T_src_dst)
      : vertex_src_(vertex_src),
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
};  // struct VLCEdge

struct MatchIsland {
  MatchIsland()
      : start_id_(0),
        end_id_(0),
        island_score_(0),
        best_id_(0),
        best_score_(0) {}

  MatchIsland(const size_t& start, const size_t& end)
      : start_id_(start),
        end_id_(end),
        island_score_(0),
        best_id_(0),
        best_score_(0) {}

  MatchIsland(const size_t& start, const size_t& end, const double& score)
      : start_id_(start),
        end_id_(end),
        island_score_(score),
        best_id_(0),
        best_score_(0) {}

  inline bool operator<(const MatchIsland& other) const {
    return island_score_ < other.island_score_;
  }

  inline bool operator>(const MatchIsland& other) const {
    return island_score_ > other.island_score_;
  }

  inline size_t size() const { return end_id_ - start_id_ + 1; }

  inline void clear() {
    start_id_ = 0;
    end_id_ = 0;
    island_score_ = 0;
    best_id_ = 0;
    best_score_ = 0;
  }

  size_t start_id_;
  size_t end_id_;
  double island_score_;
  size_t best_id_;
  double best_score_;
};  // struct MatchIsland

struct LcdTpParams {
  int max_nrFrames_between_queries_;
  int max_nrFrames_between_islands_;
  int min_temporal_matches_;
  int max_intraisland_gap_;
  int min_matches_per_island_;
};

typedef std::map<VertexID, VLCFrame, std::less<VertexID>> VLCFrameDict;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

}  // namespace kimera_distributed