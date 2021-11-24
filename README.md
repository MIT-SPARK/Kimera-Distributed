# Kimera-Distributed

This is the work-in-progress repository for distributed multirobot [Kimera](https://github.com/MIT-SPARK/Kimera). This package implements a fully distributed architecture where individual robots use peer-to-peer communication to perform inter-robot place recognition, loop closure detection, and outlier loop closure rejection. The resulting pose graph is sent to [DPGO](https://gitlab.com/mit-acl/dpgo/dpgo) to initiate distributed pose graph optimization.  

To use this package, use the `feature/kimera_distributed` branch in `Kimera-VIO`, `feature/kimera_distributed` in `Kimera-VIO-ROS`, `feature/multirobot_initialization` in `Kimera-RPGO`, `master` branch in `Kimera-PGMO`, and `feature/kimera_pgmo` branch in `Kimera-Semantics`. 

Note that we are also using a forked version of the [image-pipeline package](https://github.com/yunzc/image_pipeline/tree/feature/kimera_distributed).
Clone and checkout the `feature/kimera_distributed` branch.

Note: doing a full `catkin build` might cause problems with some of the dependencies of Kimera-Semantics, so it is recommended to build the modules one by one. 
```bash
catkin build kimera_vio_ros depth_image_proc image_undistort kimera_semantics_ros kimera_pgmo dpgo_ros
```

### Euroc datasets

Launch stack on three robots: 
```
roslaunch kimera_distributed kimera_three_robot.launch dataset_name:=Euroc
```

Play the rosbags for Euroc sequences:
```
roslaunch kimera_distributed euroc_three_robot_rosbag.launch
```

### DCIST simulator datasets
Recorded datasets can be downloaded from this [folder](https://drive.google.com/drive/folders/1WBEidZuQsKUxPYG-hcQAQe6fNH7j146M?usp=sharing).

Launch stack on three robots: 
```
roslaunch kimera_distributed kimera_three_robot.launch dataset_name:=warty
```

Play the rosbags (substitue city for camp or medfield_sim)
```
roslaunch kimera_distributed city_three_robot_rosbag.launch
```

### Logging and Debugging 
Loop closure and trajectory information will be saved in the `logs` folder. You might have to create folders with the robot names (ex. `kimera0`, `kimera1`) before 
running. 

To save the optimized mesh: 
```
rosservice call /kimera0/kimera_pgmo_node/save_mesh
```
And substitue `kimera0` for your robot name. 


### Running Pre-Set Configurations
To test Kimera-Distributed on the dataset from the DCIST integration exercise in GQ in November 2021, simply update the rosbag paths and do
```
tmuxp load gq-mout.yaml
```

## Notes
1. When running stereo dense reconstruction (see `kimera_vio_ros.launch`), you might get an error like `[ERROR] [1538875392.391423846]: Image P matrices must match (excluding x offset)`. A fix for this is to [downgrade to gcc 6](https://tuxamito.com/wiki/index.php/Installing_newer_GCC_versions_in_Ubuntu). 
2. If you get weird segmentation faults like 
```
#0  __GI___libc_free (mem=0x20) at malloc.c:3103
#1  0x00007ffff16ffcb4 in gtsam::noiseModel::Diagonal::Sigmas(Eigen::Matrix<double, -1, 1, 0, -1, 1> const&, bool) () from /usr/local/lib/libgtsam.so.4
```
that means you an Eigen conflict between the packages. Try using system Eigen for everything. 
3. Explanations for the parameters can be found in the relative repositories such as [Kimera PGMO](https://github.mit.edu/SPARK/Kimera-PGMO) and [Kimera-Semantics](https://github.mit.edu/SPARK/Kimera-Semantics)
