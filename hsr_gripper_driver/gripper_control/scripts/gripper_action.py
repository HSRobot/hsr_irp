#! /usr/bin/env python

import roslib
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from gripper_control.srv import *
import time

close_client = None
open_client = None

def on_goal(goal_handle):
    print("in on_goal")
    goal_handle.set_accepted()
    traj = goal_handle.get_goal().trajectory
    print(traj)
    open_gripper =  False
    if traj.points[1].positions[3] == 0.0:
            open_gripper = True

    if open_gripper:
        print("open gripper")
        open_client(500)
    else:
        print("close gripper")
        close_client(500, 100)

    time.sleep(1)
    goal_handle.set_succeeded()

def on_cancel():
	print("in on_cache")

if __name__ == '__main__':
    rospy.init_node('gripper_action_server')
    global close_client
    global open_client
    server = actionlib.ActionServer("gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction,
                                             on_goal, on_cancel, auto_start=False)

    rospy.wait_for_service('gripper_open')
    rospy.wait_for_service('gripper_close')

    print("server is ok")

    open_client = rospy.ServiceProxy('gripper_open', open_srv)
    close_client = rospy.ServiceProxy('gripper_close', close_srv)

    server.start()
    rospy.spin()
