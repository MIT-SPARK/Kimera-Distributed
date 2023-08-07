//
// Created by yulun on 3/19/22.
//

#include "kimera_distributed/SubmapAtlas.h"
#include <glog/logging.h>
#include <ros/console.h>

namespace kimera_distributed {

SubmapAtlas::SubmapAtlas(const Parameters& params) : params_(params) {}

SubmapAtlas::Parameters SubmapAtlas::params() const { return params_; }

int SubmapAtlas::numKeyframes() const { return (int)keyframes_.size(); }

int SubmapAtlas::numSubmaps() const { return (int)submaps_.size(); }

std::shared_ptr<Keyframe> SubmapAtlas::createKeyframe(
    int keyframe_id,
    const gtsam::Pose3& T_odom_keyframe,
    const uint64_t& timestamp) {
  if (hasKeyframe(keyframe_id)) {
    ROS_WARN_STREAM("Keyframe " << keyframe_id << " already exists!");
    return getKeyframe(keyframe_id);
  }

  auto keyframe = std::make_shared<Keyframe>(keyframe_id, timestamp);
  keyframe->setPoseInOdomFrame(T_odom_keyframe);
  keyframes_.emplace(keyframe_id, keyframe);

  // Create a new submap if needed
  if (shouldCreateNewSubmap()) {
    // Create a new submap with the same pose as the keyframe in the odom frame
    int new_submap_id = numSubmaps();
    createSubmap(new_submap_id, T_odom_keyframe, timestamp);
  }

  // Add this keyframe to the submap
  auto submap = getLatestSubmap();
  const auto& T_odom_submap = submap->getPoseInOdomFrame();
  gtsam::Pose3 T_submap_keyframe = T_odom_submap.inverse() * T_odom_keyframe;
  keyframe->setPoseInSubmapFrame(T_submap_keyframe);
  keyframe->setSubmap(submap);
  submap->addKeyframe(keyframe);

  return keyframe;
}

bool SubmapAtlas::hasKeyframe(int keyframe_id) const {
  return keyframes_.find(keyframe_id) != keyframes_.end();
}

std::shared_ptr<Keyframe> SubmapAtlas::getKeyframe(int keyframe_id) {
  const auto& it = keyframes_.find(keyframe_id);
  if (it == keyframes_.end())
    return nullptr;
  else
    return it->second;
}

std::shared_ptr<Keyframe> SubmapAtlas::getKeyframeFromStamp(const uint64_t& timestamp,
                                                            const uint64_t& tolNs) {
  int best_keyframe_id = 0;
  uint64_t best_stamp_diff = 2 * tolNs;
  for (const auto& it: keyframes_) {
    const auto keyframe_id = it.first;
    const uint64_t keyframe_stamp = it.second->stamp();
    const uint64_t stamp_diff = (keyframe_stamp < timestamp) ? (timestamp-keyframe_stamp) : (keyframe_stamp-timestamp);
    if (stamp_diff <= best_stamp_diff) {
      best_stamp_diff = stamp_diff;
      best_keyframe_id = keyframe_id;
    }
  }
  if (best_stamp_diff > tolNs) {
    return nullptr;
  } else {
    return getKeyframe(best_keyframe_id);
  }
}

std::shared_ptr<Keyframe> SubmapAtlas::getLatestKeyframe() {
  if (keyframes_.empty())
    return nullptr;
  else {
    int keyframe_id = numKeyframes() - 1;
    CHECK_GE(keyframe_id, 0);
    return CHECK_NOTNULL(getKeyframe(keyframe_id));
  }
}

std::shared_ptr<Submap> SubmapAtlas::createSubmap(int submap_id,
                                                  const gtsam::Pose3& T_odom_submap,
                                                  const uint64_t& timestamp) {
  CHECK(!hasSubmap(submap_id));
  auto submap = std::make_shared<Submap>(submap_id, timestamp);
  submap->setPoseInOdomFrame(T_odom_submap);
  submaps_.emplace(submap_id, submap);

  if (submap_id == 0) {
    // Initialize world pose of the first submap
    submap->setPoseInWorldFrame(T_odom_submap);
  } else {
    auto prev_submap = CHECK_NOTNULL(getSubmap(submap_id - 1));
    // Initialize the world pose by propagating odometry
    auto T_world_prev = prev_submap->getPoseInWorldFrame();
    auto T_odom_prev = prev_submap->getPoseInOdomFrame();
    auto T_prev_curr = T_odom_prev.inverse() * T_odom_submap;
    auto T_world_curr = T_world_prev * T_prev_curr;
    submap->setPoseInWorldFrame(T_world_curr);
  }

  return submap;
}

bool SubmapAtlas::hasSubmap(int submap_id) const {
  return submaps_.find(submap_id) != submaps_.end();
}

std::shared_ptr<Submap> SubmapAtlas::getSubmap(int submap_id) {
  const auto& it = submaps_.find(submap_id);
  if (it == submaps_.end())
    return nullptr;
  else
    return it->second;
}

std::shared_ptr<Submap> SubmapAtlas::getLatestSubmap() {
  if (submaps_.empty())
    return nullptr;
  else {
    int submap_id = numSubmaps() - 1;
    CHECK_GE(submap_id, 0);
    return CHECK_NOTNULL(getSubmap(submap_id));
  }
}

bool SubmapAtlas::shouldCreateNewSubmap() {
  if (submaps_.empty()) return true;
  if (getLatestSubmap()->numKeyframes() >= params_.max_submap_size) return true;
  if (getLatestSubmap()->distance() > params_.max_submap_distance) return true;

  return false;
}

}  // namespace kimera_distributed
