# Kimera-Distributed

This is the work-in-progress repository for distributed multirobot [Kimera](https://github.com/MIT-SPARK/Kimera). This package implements a fully distributed architecture where individual robots use peer-to-peer communication to perform inter-robot place recognition, loop closure detection, and outlier loop closure rejection. The resulting pose graph is sent to [DPGO](https://gitlab.com/mit-acl/dpgo/dpgo) to initiate distributed pose graph optimization.  

To use this package, use the `feature/dcist_dataset` branch in `Kimera-VIO` and the `feature/distributed_frontend` in `Kimera-VIO-ROS`.

Launch two robots: 
```
roslaunch kimera_distributed kimera_two_robot_euroc.launch
```

Play the rosbags for Euroc sequences:
```
roslaunch kimera_distributed kimera_two_robot_euroc_rosbag.launch
```