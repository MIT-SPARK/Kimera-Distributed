# Kimera-Distributed

This is the work-in-progress repository for distributed multirobot [Kimera](https://github.com/MIT-SPARK/Kimera). This package implements a fully distributed architecture where individual robots use peer-to-peer communication to perform inter-robot place recognition, loop closure detection, and outlier loop closure rejection. The resulting pose graph is sent to [DPGO](https://gitlab.com/mit-acl/dpgo/dpgo) to initiate distributed pose graph optimization.  
