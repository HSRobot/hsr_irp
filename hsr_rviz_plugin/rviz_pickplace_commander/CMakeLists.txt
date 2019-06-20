cmake_minimum_required(VERSION 2.8.3)
project(rviz_pickplace_commander)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  serial
  hsr_gripper_driver
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   voicePickCoke.srv
   voicePickDiamond.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rviz std_msgs hsr_gripper_driver message_runtime 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
 

add_definitions(-DQT_NO_KEYWORDS)
 

qt4_wrap_cpp(MOC_FILES
  src/pickplace_pad.h
  src/MyComboBox.h
  src/SerialWdg.h
  src/GripperWdg.h
  src/OrkWdg.h
  src/PickPlaceWdg.h
)
 

set(SOURCE_FILES
  src/pickplace_pad.cpp
  src/MyComboBox.cpp
  src/SerialWdg.cpp
  src/GripperWdg.cpp
  src/OrkWdg.cpp
  src/PickPlaceWdg.cpp
  ${MOC_FILES}
)
 

add_library(${PROJECT_NAME} ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

