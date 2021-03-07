cmake_minimum_required(VERSION 3.5)
project(kimera_distributed)

SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED
  date_time
  serialization
  thread
  filesystem
  system
  regex
  timer
  chrono
)

find_package(opengv REQUIRED)
find_package(KimeraRPGO REQUIRED)
find_package(DBoW2 REQUIRED)
if(NOT TARGET DBoW2::DBoW2)
  add_library(DBoW2::DBoW2 INTERFACE IMPORTED)
  set_target_properties(DBoW2::DBoW2 PROPERTIES
  INTERFACE_LINK_LIBRARIES "${DBoW2_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${DBoW2_INCLUDE_DIRS}")
endif()

catkin_simple()


cs_add_library(${PROJECT_NAME} 
  src/DistributedLoopClosure.cpp
  src/DistributedPcm.cpp
  src/KimeraCentralized.cpp
  src/types.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  KimeraRPGO
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
  opengv
  DBoW2::DBoW2
)

cs_add_executable(kimera_distributed_loop_closure_node
  src/DistributedLoopClosureNode.cpp)
target_link_libraries(kimera_distributed_loop_closure_node ${PROJECT_NAME})

cs_add_executable(kimera_distributed_pcm_node
  src/DistributedPcmNode.cpp)
target_link_libraries(kimera_distributed_pcm_node ${PROJECT_NAME})

cs_add_executable(publish_initial_rel_pose_node
  src/PublishInitialRelPoseNode.cpp)
target_link_libraries(publish_initial_rel_pose_node ${PROJECT_NAME})

cs_add_executable(kimera_centralized_node
  src/KimeraCentralizedNode.cpp)
target_link_libraries(kimera_centralized_node ${PROJECT_NAME})

catkin_add_gtest(test_utils test/test_utils.cpp)
target_link_libraries(test_utils ${PROJECT_NAME} -ltbb)

cs_install()

cs_export()
