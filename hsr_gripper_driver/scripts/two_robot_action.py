#! /usr/bin/env python

import roslib
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from hsr_gripper_driver.srv import *
from sensor_msgs.msg import *
import time
import thread

close_client = None
open_client = None
pub_joint_states = None
joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4','joint_5','joint_6']
joint_names2 = ['R_joint_1', 'R_joint_2', 'R_joint_3', 'R_joint_4','R_joint_5','R_joint_6']
#joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4','joint_5','joint_6','R_joint_1', 'R_joint_2', 'R_joint_3', 'R_joint_4','R_joint_5','R_joint_6']
last_joint_state = [0, 0, 0, 0]
 
#"[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, R_joint_1, R_jot_2, R_joint_3, R_joint_4, R_joint_5, R_joint_6]"

def on_goal(goal_handle):

    global last_joint_state
    print("in on_goal")
    goal_handle.set_accepted()
    traj = goal_handle.get_goal().trajectory
    print(traj)

    goal_handle.set_succeeded()

def on_cancel():
	print("in on_cache")

#def publish_joint_state(tname):
#    print("start publish_joint_state")
#    pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=1)
#    r = rospy.Rate(10)
#    while not rospy.is_shutdown():
#        now = rospy.get_rostime()
#        msg = JointState()
#        msg.header.stamp = now
#        msg.header.frame_id = "From real-time state data"
#        msg.name = joint_names
#        msg.position = last_joint_state
#        pub_joint_states.publish(msg)
#        r.sleep()

def RecallRobotPose(temp):
    
    global pub_joint_states
    now = rospy.get_rostime()
    msg = JointState()
    msg.header.stamp = now
    msg.header.frame_id = "From real-time state data"
    msg.name = joint_names2
    #msg.position = [0,-1.57,3.14,0,1.57,0, 0,-1.57,3.14,0,1.57,0]
    #msg.position = [0,-1.57,3.14,0,1.57,0,]
    msg.position = temp.position
    pub_joint_states.publish(msg)

if __name__ == '__main__':
    rospy.init_node('gripper_action_server')
    #server = actionlib.ActionServer("two_controller/follow_joint_trajectory", FollowJointTrajectoryAction,
    #                                         on_goal, on_cancel, auto_start=False)

    #rospy.wait_for_service('gripper_open')
    #rospy.wait_for_service('gripper_close')
    ros_receiver = rospy.Subscriber("UR52/joint_states",JointState, RecallRobotPose)
    pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=1)
    print("server is ok")

    #open_client = rospy.ServiceProxy('gripper_open', open_srv)
    #close_client = rospy.ServiceProxy('gripper_close', close_srv)
    #server.start()
    #try:
    #    thread.start_new_thread(publish_joint_state,("publish_joint_state",))
    #except:
    #    print "Error: unable to start thread"
    rospy.spin()
