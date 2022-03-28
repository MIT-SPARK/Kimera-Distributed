/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#ifndef KIMERA_DISTRIBUTED_INCLUDE_KIMERA_DISTRIBUTED_SUBMAPATLAS_H_
#define KIMERA_DISTRIBUTED_INCLUDE_KIMERA_DISTRIBUTED_SUBMAPATLAS_H_

#include "kimera_distributed/Submap.h"

namespace kimera_distributed {

class SubmapAtlas {
 public:
  /**
   * @brief Parameters
   */
  class Parameters {
   public:
    Parameters() : max_submap_size(10), max_submap_distance(1.0) {}
    int max_submap_size;  // maximum number of keyframes in a given submap
    double max_submap_distance;  // maximum cumulative distance in a given submap
  };
  /**
   * @brief Constructor
   * @param params
   */
  SubmapAtlas(const Parameters &params);
  /**
   * @brief Get parameters
   * @return
   */
  Parameters params() const;
  /**
   * @brief Get number of keyframes
   * @return
   */
  int numKeyframes() const;
  /**
   * @brief Get number of submaps
   * @return
   */
  int numSubmaps() const;
  /**
   * @brief Create a new keyframe with the given id and pose in the odometry frame.
   * Furthermore, the newly created keyframe is added to a submap.
   * @param keyframe_id
   * @param T_odom_keyframe pose of this keyframe in the odometry frame
   * @return
   */
  std::shared_ptr<Keyframe> createKeyframe(int keyframe_id, const gtsam::Pose3 &T_odom_keyframe);
  /**
   * @brief Check if keyframe with specified ID exists.
   * @param keyframe_id
   * @return
   */
  bool hasKeyframe(int keyframe_id) const;
  /**
   * @brief Get keyframe (return nullptr if not found)
   * @param keyframe_id
   * @return
   */
  std::shared_ptr<Keyframe> getKeyframe(int keyframe_id);
  /**
   * @brief Get latest keyframe (nullptr) if submap atlas is empty
   * @return
   */
  std::shared_ptr<Keyframe> getLatestKeyframe();
  /**
   * @brief Create a new submap with the given id and pose in the odometry frame.
   * @param submap_id
   * @param T_odom_submap
   * @return
   */
  std::shared_ptr<Submap> createSubmap(int submap_id, const gtsam::Pose3 &T_odom_submap);
  /**
   * @brief Check if a submap already exists.
   * @param submap_id
   * @return
   */
  bool hasSubmap(int submap_id) const;
  /**
   * @brief Get submap (return nullptr if not found)
   * @param submap_id
   * @return
   */
  std::shared_ptr<Submap> getSubmap(int submap_id);
  /**
   * @brief Return the latest submap (null if the submap atlas is empty)
   * @return
   */
  std::shared_ptr<Submap> getLatestSubmap();
 private:
  Parameters params_;
  std::unordered_map<int, std::shared_ptr<Keyframe>> keyframes_;
  std::unordered_map<int, std::shared_ptr<Submap>> submaps_;
  /**
   * @brief Check if a new submap should be created
   * @return
   */
  bool shouldCreateNewSubmap();
};

}

#endif //KIMERA_DISTRIBUTED_INCLUDE_KIMERA_DISTRIBUTED_SUBMAPATLAS_H_
