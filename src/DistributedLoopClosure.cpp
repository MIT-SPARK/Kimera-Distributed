/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <cassert>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <kimera_distributed/DistributedLoopClosure.h>

namespace kimera_distributed {

	DistributedLoopClosure::DistributedLoopClosure(const ros::NodeHandle& n):
	nh_(n)
	{
		int my_id_int = -1;
		int num_robots_int = -1;
		assert(ros::param::get("~robot_id", my_id_int));
		assert(ros::param::get("~num_robots", num_robots_int));
		assert(my_id_int >= 0);
		assert(num_robots_int > 0);
		my_id_ = my_id_int;
		num_robots_ = num_robots_int;
		next_pose_id_ = 0;

		// Visual place recognition params
		assert(ros::param::get("~alpha", alpha_));
		assert(ros::param::get("~dist_local", dist_local_));
		assert(ros::param::get("~max_db_results", max_db_results_));
		assert(ros::param::get("~base_nss_factor", base_nss_factor_));

		// Initialize bag-of-word database
		std::string orb_vocab_path;
		assert(ros::param::get("~vocabulary_path", orb_vocab_path));
		OrbVocabulary vocab;
		vocab.load(orb_vocab_path);
		db_BoW_ = std::unique_ptr<OrbDatabase>(new OrbDatabase(vocab));

		for (size_t id = my_id_; id < num_robots_; ++id){
			std::string topic = "/kimera" + std::to_string(id) + "/kimera_vio_ros/bow_query";
			ros::Subscriber sub = nh_.subscribe(topic, 10, &DistributedLoopClosure::bowCallback, this);
			bow_subscribers.push_back(sub);
		}

		ROS_INFO_STREAM("Distributed Kimera node initialized (ID = " << my_id_ << "). \n" 
						<< "Parameters: \n" 
						<< "alpha = " << alpha_ << "\n"
						<< "dist_local = " << dist_local_ << "\n"
						<< "max_db_results = " << max_db_results_ << "\n"
						<< "base_nss_factor = " << base_nss_factor_);
	}
	

	DistributedLoopClosure::~DistributedLoopClosure() {}

	
	void DistributedLoopClosure::bowCallback(const kimera_distributed::BowQueryConstPtr& msg) {

		RobotID robot_id = msg->robot_id;
		assert(robot_id >= my_id_);
		PoseID pose_id = msg->pose_id;
		VertexID query_vertex_id = std::make_pair(robot_id, pose_id);
		DBoW2::BowVector bow_vec;
		BowVectorFromMsg(msg->bow_vector, &bow_vec);

		double nss_factor = base_nss_factor_;
		int max_possible_match_id = static_cast<int>(next_pose_id_) - 1;
  		if (robot_id == my_id_){
  			max_possible_match_id -= dist_local_;
  			nss_factor = db_BoW_->getVocabulary()->score(bow_vec, latest_bowvec_);
  		}
  		if (max_possible_match_id < 0) max_possible_match_id = 0;

  		ROS_WARN_STREAM(nss_factor);
  		DBoW2::QueryResults query_result;
  		db_BoW_->query(bow_vec, query_result, max_db_results_, max_possible_match_id);

  		if (!query_result.empty()){
  			DBoW2::Result best_result = query_result[0]; 
  			if (best_result.Score >= alpha_ * nss_factor){
				VertexID my_vertex_id = std::make_pair(my_id_, best_result.Id);
				
  				ROS_WARN_STREAM("Detected potential loop closure between " 
  								<< "(" << robot_id << ", " << pose_id << ")" 
  								<< " and " 
  								<< "(" << my_id_ << ", " << best_result.Id << ")" 
  								);
  				
  			}
  		}

  		// Add Bag-of-word vector to database
		if (robot_id == my_id_){
			assert(pose_id == next_pose_id_);
			assert(db_BoW_->add(bow_vec) == next_pose_id_);
			latest_bowvec_ = bow_vec;
			next_pose_id_++;
		}
  		
	}

}