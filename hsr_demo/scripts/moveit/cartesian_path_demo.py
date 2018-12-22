#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_cartesian_path_demo', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

waypoints = []

# start with the current pose
#waypoints.append(group.get_current_pose().pose)
start_pose = group.get_current_pose().pose
# first orient gripper and move forward (+x)
wpose = geometry_msgs.msg.Pose()

'''
wpose.position.x = -0.512873527464	
wpose.position.y = 0.410581743076
wpose.position.z = 0.50244383546
'''

wpose.position.x = start_pose.position.x - 0.2 
wpose.position.y = start_pose.position.y
wpose.position.z = start_pose.position.z + 0.2
wpose.orientation = start_pose.orientation

'''
wpose.orientation.w = 0.0482532222573 
wpose.orientation.x = 0.254673315982
wpose.orientation.y = 0.247313977335
wpose.orientation.z = -0.933621403612
'''
group.set_pose_target(wpose)
group.go(wait=True)



#rospy.sleep(5)
