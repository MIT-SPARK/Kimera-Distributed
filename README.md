# Kimera-Distributed

## Introduction
**Kimera-Distributed** is a currently private repository of [Kimera-Multi](https://arxiv.org/abs/2106.14386) that contains the following modules: 
1. Distributed loop closure detection 
2. Pairwise consistency maximization (PCM)
3. Config and launch files to launch the entire Kimera-Multi stack

Refer to the following publications for more information:

 - Y. Chang, Y. Tian, J. P. How and L. Carlone, "Kimera-Multi: a System for Distributed Multi-Robot Metric-Semantic Simultaneous Localization and Mapping," IEEE International Conference on Robotics and Automation (ICRA), 2021.
 
 - Y. Tian, Y. Chang, F. Herrera Arias, C.Nieto-Granda, J. P. How, L. Carlone,  "Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems," IEEE Transactions on Robotics (T-RO), conditionally accepted, 2021.

## Dependencies 
Currently using the private (mit.edu) repo of the following repositories:

[Kimera-VIO](https://github.mit.edu/SPARK/Kimera-VIO) branch: feature/kimera_distributed

[Kimera-VIO-ROS](https://github.mit.edu/SPARK/Kimera-VIO-ROS) branch: feature/kimera_distributed

[Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO) branch: feature/multirobot_initialization

[pose_graph_tools](https://github.mit.edu/SPARK/pose_graph_tools) branch: master

[Kimera-Semantics](https://github.mit.edu/SPARK/Kimera-Semantics) branch: master

[Kimera-PGMO](https://github.mit.edu/SPARK/Kimera-PGMO) branch: master

[Kimera-Multi-LCD](https://github.mit.edu/SPARK/Kimera-Multi-LCD) branch: master

[dpgo](https://gitlab.com/mit-acl/dpgo/dpgo) branch: master

[dpgo_ros](https://gitlab.com/mit-acl/dpgo/dpgo_ros) branch: master

## Recommended Usage

On each robot, first launch Kimera-VIO-ROS. Then, use the `kimera_distributed.launch` provided in this repo to launch distributed loop closure, PCM, and dpgo:
```
roslaunch kimera_distributed kimera_distributed.launch robot_name:=sobek robot_id:=0 num_robots:=3 dataset_name:=Jackal use_actionlib:=false multi_master:=false
```

**Explaination of launch file args**
- *robot_id*: unique integer ID associated with this robot
- *num_robots*: total number of robots in the team
- *robot_name*: unique ROS namespace associated with this robot
- *dataset_name*: parameter settings (e.g., "Euroc" for EuRoc dataset, "Jackal" for MIT jackals)
- *use_actionlib*: set to true to use actionlib for inter-robot communication (otherwise, use ROS service)
- *multi_master*: set to true to launch reliable UDP node to relay inter-robot messages. For additional information about reliable UDP, refer to this [online wiki page](https://github.mit.edu/SPARK/Kimera-Distributed/wiki/Running-on-multiple-multiple-ROS-Masters-with-Reliable-UDP) .

**Example used in Nov 2021 GQ tests**

A complete example is provided in `config/gt-mout-three-robot.yaml`, which is used for post processing of Graces Quater experiments (Nov 2021). To run the example:

```
tmuxp load config/gq-mout-three-robot.yaml
```

## Running on Datasets
The following contains instructions for running on benchmark datasets, custom datasets, and simulated datasets. 

#### Euroc datasets

Launch stack on three robots: 
```
roslaunch kimera_distributed kimera_three_robot.launch dataset_name:=Euroc
```

Play the rosbags for Euroc sequences:
```
roslaunch kimera_distributed euroc_three_robot_rosbag.launch
```

#### Medfield Datasets
Launch stack on three robots: 
```
roslaunch kimera_distributed kimera_three_robot.launch dataset_name:=JackalMedfield
```

Play the rosbags for Euroc sequences:
```
roslaunch kimera_distributed rosbag_medfield.launch
```

#### DCIST simulator datasets
Recorded datasets can be downloaded from this [folder](https://drive.google.com/drive/folders/1WBEidZuQsKUxPYG-hcQAQe6fNH7j146M?usp=sharing).

Launch stack on three robots: 
```
roslaunch kimera_distributed kimera_three_robot.launch dataset_name:=warty
```

Play the rosbags (substitue city for camp or medfield_sim)
```
roslaunch kimera_distributed city_three_robot_rosbag.launch
```

#### Logging and Debugging 
Loop closure and trajectory information will be saved in the `logs` folder. You might have to create folders with the robot names (ex. `kimera0`, `kimera1`) before 
running. 

To save the optimized mesh: 
```
rosservice call /kimera0/kimera_pgmo_node/save_mesh
```
And substitue `kimera0` for your robot name. 

## Notes
1. When running stereo dense reconstruction (see `kimera_vio_ros.launch`), you might get an error like `[ERROR] [1538875392.391423846]: Image P matrices must match (excluding x offset)`. A fix for this is to [downgrade to gcc 6](https://tuxamito.com/wiki/index.php/Installing_newer_GCC_versions_in_Ubuntu). 
2. If you get weird segmentation faults like 
```
#0  __GI___libc_free (mem=0x20) at malloc.c:3103
#1  0x00007ffff16ffcb4 in gtsam::noiseModel::Diagonal::Sigmas(Eigen::Matrix<double, -1, 1, 0, -1, 1> const&, bool) () from /usr/local/lib/libgtsam.so.4
```
that means you an Eigen conflict between the packages. Try using system Eigen for everything. 
3. Explanations for the parameters can be found in the relative repositories such as [Kimera PGMO](https://github.mit.edu/SPARK/Kimera-PGMO) and [Kimera-Semantics](https://github.mit.edu/SPARK/Kimera-Semantics)
