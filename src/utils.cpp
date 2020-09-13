/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <cassert>
#include <ros/console.h>
#include <kimera_distributed/utils.h>

namespace kimera_distributed {

void BowVectorToMsg(const DBoW2::BowVector& bow_vec, kimera_distributed::BowVector* msg) 
{
  msg->word_ids.clear();
  msg->word_values.clear();
  for (auto it = bow_vec.begin(); it != bow_vec.end(); ++it) {
    msg->word_ids.push_back(it->first);
    msg->word_values.push_back(it->second);
  }
}

void BowVectorFromMsg(const kimera_distributed::BowVector& msg, DBoW2::BowVector* bow_vec)
{
  assert(msg.word_ids.size() == msg.word_values.size());
  bow_vec->clear();
  for (size_t i = 0; i < msg.word_ids.size(); ++i){
    bow_vec->addWeight(msg.word_ids[i], msg.word_values[i]);
  }
}

}  // namespace kimera_distributed