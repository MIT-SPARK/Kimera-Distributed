/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <ros/console.h>
#include <map>
#include <string>
#include <kimera_distributed/BowQuery.h>
#include <kimera_distributed/BowVector.h>
#include <DBoW2/DBoW2.h>

namespace kimera_distributed {
    void BowVectorToMsg(const DBoW2::BowVector& bow_vec, kimera_distributed::BowVector* msg);
    void BowVectorFromMsg(const kimera_distributed::BowVector& msg, DBoW2::BowVector* bow_vec);

}  // namespace kimera_distributed
