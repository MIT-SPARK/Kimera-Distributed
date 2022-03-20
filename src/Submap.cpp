/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include "kimera_distributed/Submap.h"

namespace kimera_distributed {

Submap::Submap(int id) : id_(id) {}

int Submap::id() const { return id_; }

int Submap::numKeyframes() const { return (int) keyframes_.size(); }

gtsam::Pose3 Submap::getPoseInOdomFrame() const {
  return T_odom_submap_;
}

void Submap::setPoseInOdomFrame(const gtsam::Pose3 &T_odom_submap) {
  T_odom_submap_ = T_odom_submap;
}

void Submap::addKeyframe(const std::shared_ptr<Keyframe> &keyframe) {
  keyframes_.emplace(keyframe->id(), keyframe);
}

std::shared_ptr<Keyframe> Submap::getKeyframe(int keyframe_id) const {
  const auto it = keyframes_.find(keyframe_id);
  if (it == keyframes_.end()) {
    return nullptr;
  } else {
    return it->second;
  }
}

std::unordered_set<int> Submap::getKeyframeIDs() const {
  std::unordered_set<int> keyframe_ids;
  for (const auto &it : keyframes_)
    keyframe_ids.emplace(it.first);
  return keyframe_ids;
}

}

