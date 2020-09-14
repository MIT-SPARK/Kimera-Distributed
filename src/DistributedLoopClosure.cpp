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
		assert(ros::param::get("/num_robots", num_robots_int));
		assert(my_id_int >= 0);
		assert(num_robots_int > 0);
		my_id_ = my_id_int;
		num_robots_ = num_robots_int;
		

		for (size_t id = my_id_; id < num_robots_; ++id){
			std::string topic = "/kimera" + std::to_string(id) + "/kimera_vio_ros/bow_query";
			ros::Subscriber sub = nh_.subscribe(topic, 10, &DistributedLoopClosure::bowCallback, this);
			bow_subscribers.push_back(sub);
		}

		ROS_INFO_STREAM("Distributed Kimera node initialized (ID = " << my_id_ << ")");
	}
	DistributedLoopClosure::~DistributedLoopClosure() {}



	void DistributedLoopClosure::bowCallback(const kimera_distributed::BowQueryConstPtr& msg) {
	}

}