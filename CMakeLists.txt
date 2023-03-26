cmake_minimum_required(VERSION 3.5)
project(kimera_distributed)

SET(CMAKE_BUILD_TYPE Release)

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

find_package(GTSAM REQUIRED)
find_package(DBoW2 REQUIRED)
if(NOT TARGET DBoW2::DBoW2)
  add_library(DBoW2::DBoW2 INTERFACE IMPORTED)
  set_target_properties(DBoW2::DBoW2 PROPERTIES
  INTERFACE_LINK_LIBRARIES "${DBoW2_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${DBoW2_INCLUDE_DIRS}")
endif()

catkin_simple()


cs_add_library(${PROJECT_NAME}
  src/Submap.cpp
  src/SubmapAtlas.cpp
  src/DistributedLoopClosure.cpp
  src/DistributedLoopClosureRos.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
  DBoW2::DBoW2
)

cs_add_executable(kimera_distributed_loop_closure_node
  src/DistributedLoopClosureNode.cpp)
target_link_libraries(kimera_distributed_loop_closure_node ${PROJECT_NAME})

# Unit tests 
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

catkin_add_gtest(${PROJECT_NAME}-test_utils test/test_utils.cpp)
target_link_libraries(${PROJECT_NAME}-test_utils ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test_submap test/test_submap.cpp)
target_link_libraries(${PROJECT_NAME}-test_submap ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test_submap_atlas test/test_submap_atlas.cpp)
target_link_libraries(${PROJECT_NAME}-test_submap_atlas ${PROJECT_NAME})


cs_install()

cs_export()
