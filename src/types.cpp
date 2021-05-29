/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <kimera_distributed/types.h>
#include <kimera_distributed/utils.h>

namespace kimera_distributed {

VLCFrame::VLCFrame() {}

VLCFrame::VLCFrame(const RobotID& robot_id,
                   const PoseID& pose_id,
                   const std::vector<gtsam::Vector3>& keypoints_3d,
                   const std::vector<gtsam::Vector3>& versors,
                   const OrbDescriptor& descriptors_mat)
    : robot_id_(robot_id),
      pose_id_(pose_id),
      keypoints_(keypoints_3d),
      versors_(versors),
      descriptors_mat_(descriptors_mat) {
  assert(keypoints_.size() == descriptors_mat_.size().height);
  assert(versors_.size() == descriptors_mat_.size().height);
  initializeDescriptorsVector();
}

void VLCFrame::initializeDescriptorsVector() {
  descriptors_vec_.clear();
  // Create vector of descriptors
  int L = descriptors_mat_.size().width;
  descriptors_vec_.resize(descriptors_mat_.size().height);

  for (size_t i = 0; i < descriptors_vec_.size(); i++) {
    descriptors_vec_[i] =
        cv::Mat(1, L, descriptors_mat_.type());  // one row only
    descriptors_mat_.row(i).copyTo(descriptors_vec_[i].row(0));
  }
}

}  // namespace kimera_distributed