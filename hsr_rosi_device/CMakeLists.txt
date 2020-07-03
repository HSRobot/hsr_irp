cmake_minimum_required(VERSION 2.8.3)
project(hsr_rosi_device)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  industrial_robot_client
  simple_message
)
add_definitions(-DLINUXSOCKETS=1)  #build using LINUX SOCKETS libraries


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ClearFaultSrv.srv
   SetEnableSrv.srv
   StopMoveSrv.srv
   setModeSrv.srv
   getModeSrv.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS roscpp industrial_robot_client simple_message 
  DEPENDS system_lib
  message_runtime
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## 编译库
set(SRC_FILES src/joint_trajectory_full_downloader.cpp
              src/joint_trajectory_full_interface.cpp
              src/joint_trajectory_cubic_downloader.cpp
              src/robot_sys_message.cpp
			)

add_library(hsr_rosi_device ${SRC_FILES})
add_dependencies(hsr_rosi_device ${catkin_EXPORTED_TARGETS} ${hsr_rosi_device_EXPORTED_TARGETS})
target_link_libraries(hsr_rosi_device simple_message)

#编译可执行程序
add_executable(motion_rosi_download_full_interface
	src/generic_full_joint_downloader_node.cpp)

add_executable(motion_rosi_download_full_Imp_interface
        src/generic_joint_downloader_imp_node.cpp)

target_link_libraries(motion_rosi_download_full_Imp_interface
        industrial_robot_client
        hsr_rosi_device
        simple_message
        ${catkin_LIBRARIES})


add_dependencies(motion_rosi_download_full_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_rosi_download_full_interface
  industrial_robot_client
  hsr_rosi_device
  simple_message
  ${catkin_LIBRARIES})

#编译可执行程序
add_executable(motion_rosi_download_cubic_interface
        src/generic_cubic_joint_downloader_node.cpp)
add_dependencies(motion_rosi_download_cubic_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_rosi_download_cubic_interface
  industrial_robot_client
  hsr_rosi_device
  simple_message
  ${catkin_LIBRARIES})

