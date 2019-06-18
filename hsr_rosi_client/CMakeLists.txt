cmake_minimum_required(VERSION 2.8.3)
project(hsr_rosi_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  industrial_robot_client
  simple_message
)
add_definitions(-DLINUXSOCKETS=1)  #build using LINUX SOCKETS libraries

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS roscpp industrial_robot_client simple_message 
  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## 编译库
set(SRC_FILES src/joint_trajectory_full_downloader.cpp
              src/joint_trajectory_full_interface.cpp
              src/joint_trajectory_cubic_downloader.cpp
			)

add_library(hsr_robot_client ${SRC_FILES})
target_link_libraries(hsr_robot_client simple_message)

#编译可执行程序
add_executable(motion_download_full_interface
	src/generic_full_joint_downloader_node.cpp)
target_link_libraries(motion_download_full_interface
  industrial_robot_client
  hsr_robot_client 
  simple_message
  ${catkin_LIBRARIES})

#编译可执行程序
add_executable(motion_download_cubic_interface
        src/generic_cubic_joint_downloader_node.cpp)
target_link_libraries(motion_download_cubic_interface
  industrial_robot_client
  hsr_robot_client
  simple_message
  ${catkin_LIBRARIES})

