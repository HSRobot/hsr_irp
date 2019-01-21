cmake_minimum_required(VERSION 2.8.3)
project(hsr_moveit_api)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  hsr_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_perception
)


find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})



catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs hsr_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_api src/base_api.cpp)

add_dependencies(base_api  ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(base_api 
   ${catkin_LIBRARIES}
 )

