/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#ifndef KIMERA_DISTRIBUTED_INCLUDE_KIMERA_DISTRIBUTED_KEYFRAME_H_
#define KIMERA_DISTRIBUTED_INCLUDE_KIMERA_DISTRIBUTED_KEYFRAME_H_

#include <gtsam/geometry/Pose3.h>
#include <memory>

namespace kimera_distributed {

// Forward declaration
class Submap;

class Keyframe {
 public:
  /**
   * @brief Constructor
   * @param keyframe_id unique ID of this keyframe
   * @param submap pointer to the submap that contains this keyframe
   */
  Keyframe(int id) : id_(id) {}
  /**
   * @brief Get keyframe ID
   * @return
   */
  int id() const { return id_; }
  /**
   * @brief Get submap
   * @return
   */
  std::shared_ptr<Submap> getSubmap() const {
    return submap_;
  }
  /**
   * @brief Set submap
   * @param submap
   */
  void setSubmap(const std::shared_ptr<Submap> &submap) {
    submap_ = submap;
  }
  /**
   * @brief Get the pose of this frame in the submap reference frame
   * @return
   */
  gtsam::Pose3 getPoseInSubmapFrame() const {
    return T_submap_keyframe_;
  }
  /**
   * @brief Set the pose of this keyframe in the submap frame
   * @param T_submap_keyframe
   */
  void setPoseInSubmapFrame(const gtsam::Pose3 &T_submap_keyframe) {
    T_submap_keyframe_ = T_submap_keyframe;
  }
  /**
   * @brief Get the pose of this keyframe in the odometry frame
   * @return
   */
  gtsam::Pose3 getPoseInOdomFrame() const {
    return T_odom_keyframe_;
  }
  /**
   * @brief Set pose of this keyframe in the odometry frame
   * @param T_odom_keyframe
   */
  void setPoseInOdomFrame(const gtsam::Pose3 &T_odom_keyframe) {
    T_odom_keyframe_ = T_odom_keyframe;
  }
 private:
  const int id_;  // unique id associated with this keyframe
  std::shared_ptr<Submap> submap_;  // pointer to the submap that contains this KF
  gtsam::Pose3 T_odom_keyframe_;    // pose of this keyframe in the odometry frame
  gtsam::Pose3 T_submap_keyframe_;  // pose of this keyframe in the submap frame
};

}

#endif //KIMERA_DISTRIBUTED_INCLUDE_KIMERA_DISTRIBUTED_KEYFRAME_H_
