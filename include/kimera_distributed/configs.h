/*
 * Copyright Notes
 *
 * Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <kimera_multi_lcd/types.h>

namespace kimera_distributed {

struct DistributedLoopClosureConfig {
  size_t my_id_;
  size_t num_robots_;
  std::string frame_id_;
  bool log_output_;
  std::string log_output_dir_;

  // Offline mode
  bool run_offline_;
  std::string offline_dir_;

  // LCD Params
  kimera_multi_lcd::LcdParams lcd_params_;

  // Submap Params
  SubmapAtlas::Parameters submap_params_;

  // Parameters controlling communication due to VLC request/response
  int bow_batch_size_;   // Maximum number of Bow vectors per robot to request in one
                         // batch
  int vlc_batch_size_;   // Maximum number of VLC frames per robot to request in one
                         // batch
  int loop_batch_size_;  // Maximum number of loop closures to synchronize in one batch
  int comm_sleep_time_;  // Sleep time of communication thread
  int loop_sync_sleep_time_;  // Sleep time of loop synchronization
  int detection_batch_size_;  // Maximum number of loop detection to perform in one
                              // batch
  int bow_skip_num_;          // Request every bow_skip_num_ bow vectors

  // Robot names
  std::map<size_t, std::string> robot_names_;
};

}  // namespace kimera_distributed