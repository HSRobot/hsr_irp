#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose

REFERENCE_FRAME = 'world'

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_cartesian_path_demo', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")


rospy.sleep(1)
base_table_id = 'base_table'   
base_table_size = [1.2, 1.2, 0.01]
base_table_pose = PoseStamped()
base_table_pose.header.frame_id = REFERENCE_FRAME
base_table_pose.pose.position.x = 0.0
base_table_pose.pose.position.y = 0.0
base_table_pose.pose.position.z = 0 - 0.01 / 2.0
base_table_pose.pose.orientation.w = 1.0

rospy.sleep(1)

scene.add_box(base_table_id, base_table_pose, base_table_size)
