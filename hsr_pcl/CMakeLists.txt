cmake_minimum_required(VERSION 2.8.3)
project(hsr_pcl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_ros
  roscpp
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
  LoadPCD.srv
  SavePCD.srv
)

generate_messages(
DEPENDENCIES
  roscpp
)

catkin_package(
	CATKIN_DEPENDS pcl_ros roscpp message_runtime
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_demo src/pcl_demo.cpp)
add_executable(kinect_filter src/kinect_filter.cpp)

add_dependencies(kinect_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(pcl_demo
  ${catkin_LIBRARIES}
 )

target_link_libraries(kinect_filter
 ${catkin_LIBRARIES}
)


