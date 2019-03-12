#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import random

REFERENCE_FRAME = 'world'

TABLE_GROUND = 0.25

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('loop_test', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

# Give the scene a chance to catch up    
rospy.sleep(2)

# Give each of the scene objects a unique name
base_table_id = 'base_table'
left_wall_id = 'left_wall'
right_wall_id = 'right_wall'
foreward_wall_id = 'foreward_wall'
backward_wall_id = 'backward_wall'

# Remove leftover objects from a previous run
scene.remove_world_object(base_table_id)
scene.remove_world_object(left_wall_id)
scene.remove_world_object(right_wall_id)
scene.remove_world_object(foreward_wall_id)
scene.remove_world_object(backward_wall_id)

# Set the dimensions of the scene objects [l, w, h]
base_table_size = [3, 3, 0.01]
left_wall_size = [3, 0.01, 3]
right_wall_size = [3, 0.01, 3]
foreward_wall_size = [0.01, 3, 3]
backward_wall_size = [0.01, 3, 3]

# Add a base table to the scene
base_table_pose = PoseStamped()
base_table_pose.header.frame_id = REFERENCE_FRAME
base_table_pose.pose.position.x = 0.0
base_table_pose.pose.position.y = 0.0
#base_table_pose.pose.position.z = TABLE_GROUND - base_table_size[2] / 2.0

base_table_pose.pose.position.z = TABLE_GROUND +0.5#-0.45
base_table_pose.pose.orientation.w = 1.0
scene.add_box(base_table_id, base_table_pose, base_table_size)

# Add a left_wall to the scene
left_wall_pose = PoseStamped()
left_wall_pose.header.frame_id = REFERENCE_FRAME
left_wall_pose.pose.position.x = 0.0
left_wall_pose.pose.position.y = -1.0
left_wall_pose.pose.position.z = TABLE_GROUND
left_wall_pose.pose.orientation.w = 1.0
scene.add_box(left_wall_id, left_wall_pose, left_wall_size)

# Add a right_wall to the scene
right_wall_pose = PoseStamped()
right_wall_pose.header.frame_id = REFERENCE_FRAME
right_wall_pose.pose.position.x = 0.0
right_wall_pose.pose.position.y = 1.0
right_wall_pose.pose.position.z = TABLE_GROUND
right_wall_pose.pose.orientation.w = 1.0
scene.add_box(right_wall_id, right_wall_pose, right_wall_size)

# Add a foreward_wall to the scene
foreward_wall_pose = PoseStamped()
foreward_wall_pose.header.frame_id = REFERENCE_FRAME
foreward_wall_pose.pose.position.x = 1.0
foreward_wall_pose.pose.position.y = 0.0
foreward_wall_pose.pose.position.z = TABLE_GROUND
foreward_wall_pose.pose.orientation.w = 1.0
scene.add_box(foreward_wall_id, foreward_wall_pose, foreward_wall_size)

# Add a backward_wall to the scene
backward_wall_pose = PoseStamped()
backward_wall_pose.header.frame_id = REFERENCE_FRAME
backward_wall_pose.pose.position.x = -1.0
backward_wall_pose.pose.position.y = 0.0
backward_wall_pose.pose.position.z = TABLE_GROUND
backward_wall_pose.pose.orientation.w = 1.0
scene.add_box(backward_wall_id, backward_wall_pose, backward_wall_size)

# Give the scene a chance to catch up    
rospy.sleep(2)
group.set_max_velocity_scaling_factor(0.2)
while not rospy.core.is_shutdown():
	group.set_random_target()
	group.go(wait=True)
	rospy.sleep(random.randint(1, 10))
