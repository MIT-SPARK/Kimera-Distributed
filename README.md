# Kimera-Distributed

This is the work-in-progress repository for distributed multirobot [Kimera](https://github.com/MIT-SPARK/Kimera). This package implements a fully distributed architecture where individual robots use peer-to-peer communication to perform inter-robot place recognition, loop closure detection, and outlier loop closure rejection. The resulting pose graph is sent to [DPGO](https://gitlab.com/mit-acl/dpgo/dpgo) to initiate distributed pose graph optimization.  

To use this package, use the `feature/dcist_dataset` branch in `Kimera-VIO` and the `feature/distributed_frontend` in `Kimera-VIO-ROS`.

### Euroc datasets

Launch two robots: 
```
roslaunch kimera_distributed kimera_two_robot.launch dataset_name:=Euroc
```

Play the rosbags for Euroc sequences:
```
roslaunch kimera_distributed kimera_two_robot_euroc_rosbag.launch
```

### DCIST simulator datasets
Launch two robots: 
```
roslaunch kimera_distributed kimera_two_robot.launch dataset_name:=warty
```

Play the rosbags:
```
roslaunch kimera_distributed kimera_two_robot_dcist_rosbag.launch
```