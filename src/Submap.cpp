/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include "kimera_distributed/Submap.h"

namespace kimera_distributed {

Submap::Submap(int id, uint64_t stamp) : id_(id), stamp_(stamp), distance_(0) {}

int Submap::id() const { return id_; }

uint64_t Submap::stamp() const { return stamp_; }

int Submap::numKeyframes() const { return (int) keyframes_.size(); }

double Submap::distance() const { return distance_; }

gtsam::Pose3 Submap::getPoseInOdomFrame() const {
  return T_odom_submap_;
}

void Submap::setPoseInOdomFrame(const gtsam::Pose3 &T_odom_submap) {
  T_odom_submap_ = T_odom_submap;
}

gtsam::Pose3 Submap::getPoseInWorldFrame() const {
  return T_world_submap_;
}

void Submap::setPoseInWorldFrame(const gtsam::Pose3 &T_world_submap) {
  T_world_submap_ = T_world_submap;
}

void Submap::addKeyframe(const std::shared_ptr<Keyframe> &keyframe) {
  keyframes_.emplace(keyframe->id(), keyframe);
  // Update distance
  const auto keyframe_prev = getKeyframe(keyframe->id() - 1);
  if (keyframe_prev) {
    const auto T_submap_prev = keyframe_prev->getPoseInSubmapFrame();
    const auto T_submap_curr = keyframe->getPoseInSubmapFrame();
    const auto T_prev_curr = (T_submap_prev.inverse()) * T_submap_curr;
    distance_ += T_prev_curr.translation().norm();
  }
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

