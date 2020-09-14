/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */
#pragma once

// Includes
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

#include <kimera_distributed/DistributedLoopClosure.h>


namespace kimera_distributed {

// Class definition
class KimeraDistributed {
 public:
	// Constructor
	KimeraDistributed();

	// Destructor
	~KimeraDistributed();

	// Define main interface functions
	bool Initialize(const ros::NodeHandle& n);

 protected:
 
 private:

};

}  // namespace kimera_multi
