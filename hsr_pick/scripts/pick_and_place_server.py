#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    moveit_pick_and_place_demo.py - Version 0.1 2014-01-14
    
    Command the gripper to grasp a target object and move it to a new location, all
    while avoiding simulated obstacles.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import threading
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

from hsr_pick.srv import *
from hsr_gripper_driver.srv import *

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

ARM_HOME_POSE = 'home'

# GRIPPER_FRAME = 'pick_gripper_link'
GRIPPER_FRAME = 'pick_gripper_link'

GRIPPER_OPEN = [0.0]
GRIPPER_CLOSED = [-0.3491]
GRIPPER_NEUTRAL = [0.0]

GRIPPER_JOINT_NAMES = ['right_finger_2_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'base_link'
open_client = None

class MoveItDemo:
    def __init__(self, pickPos, placePos):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
#        rospy.init_node('moveit_demo')
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
                        
        # Initialize the move group for the right arm
        pick_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Initialize the momitive group for the right gripper
        pick_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # Get the name of the end-effector link
        end_effector_link = pick_arm.get_end_effector_link()
 
        # Allow some leeway in position (meters) and orientation (radians)
        pick_arm.set_goal_position_tolerance(0.05)
        pick_arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        pick_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        pick_arm.set_pose_reference_frame(REFERENCE_FRAME)

        pick_arm.set_planner_id("RRTConnectkConfigDefault")        

        # Allow 5 seconds per planning attempt
        pick_arm.set_planning_time(10)
        
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5
        
        # Set a limit on the number of place attempts
        max_place_attempts = 5
                
        # Give the scene a chance to catch up
        rospy.sleep(2)

        # Give each of the scene objects a unique name
        base_table_id = 'base_table'        
        target_id = 'target'
        #tool_id = 'tool'
                
        # Remove leftover objects from a previous run
        scene.remove_world_object(base_table_id)
        #scene.remove_world_object(table_id)
        #scene.remove_world_object(box1_id)
        #scene.remove_world_object(box2_id)
        scene.remove_world_object(target_id)
        #scene.remove_world_object(tool_id)
        
        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, target_id)

        
        # Open the gripper to the neutral position
       # pick_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
       # pick_gripper.go()
       
        rospy.sleep(1)

        # Set the height of the table off the ground
        # table_ground = 0.6
        table_ground = 0.0        

        # Set the dimensions of the scene objects [l, w, h]
        base_table_size = [2, 2, 0.04]
        #table_size = [0.2, 0.7, 0.01]
        #box1_size = [0.1, 0.05, 0.05]
        #box2_size = [0.05, 0.05, 0.15]
        
        # Set the target size [l, w, h]
        target_size = [0.055, 0.055, 0.12]
        
        # Add a base table to the scene
        base_table_pose = PoseStamped()
        base_table_pose.header.frame_id = REFERENCE_FRAME
        base_table_pose.pose.position.x = 0.0
        base_table_pose.pose.position.y = 0.0
        base_table_pose.pose.position.z = -0.3
        base_table_pose.pose.orientation.w = 1.0
        scene.add_box(base_table_id, base_table_pose, base_table_size)

        # Give the scene a chance to catch up    
        rospy.sleep(1)
        
        # Start the arm in the "resting" pose stored in the SRDF file
        pick_arm.set_named_target(ARM_HOME_POSE)
        pick_arm.go()

        rospy.sleep(1)


        # Add a table top and two boxes to the scene
        #table_pose = PoseStamped()
        #table_pose.header.frame_id = REFERENCE_FRAME
        #table_pose.pose.position.x = 0.5
        #table_pose.pose.position.y = -0.4
        #table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        #table_pose.pose.orientation.w = 1.0
        #scene.add_box(table_id, table_pose, table_size)       
    
        # Set the target pose in between the boxes and on the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME

        target_pose.pose.position.x = pickPos.pose.position.x
        target_pose.pose.position.y = pickPos.pose.position.y
        target_pose.pose.position.z = pickPos.pose.position.z
        target_pose.pose.orientation.w = pickPos.pose.orientation.w
 
        # Add the target object to the scene
        scene.add_box(target_id, target_pose, target_size)
        
        # Make the table red and the boxes orange
        #self.setColor(table_id, 0.8, 0, 0, 1.0)
#        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
#        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)
        
        # Make the target yellow
        self.setColor(target_id, 0.9, 0.9, 0, 1.0)
        
        # Send the colors to the planning scene
        self.sendColors()
        
        # Set the support surface name to the table object
        #pick_arm.set_support_surface_name(table_id)
        
        # Specify a pose to place the target after being picked up 
		#指定拾取后的放置目标姿态
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = placePos.pose.position.x
        place_pose.pose.position.y = placePos.pose.position.y
        place_pose.pose.position.z = placePos.pose.position.z
        place_pose.pose.orientation.w = placePos.pose.orientation.w
        place_pose.pose.orientation.x = placePos.pose.orientation.x
        place_pose.pose.orientation.y = placePos.pose.orientation.y
        place_pose.pose.orientation.z = placePos.pose.orientation.z


        # Initialize the grasp pose to the target pose
		#初始化抓取目标点位
        grasp_pose = target_pose
        
        # Shift the grasp pose by half the width of the target to center it
        # this way is just used by PR2 robot
        #grasp_pose.pose.position.y -= target_size[1] / 2.0
                
        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, [target_id])

        # Publish the grasp poses so they can be viewed in RV
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)
    
        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        
        # Repeat until we succeed or run out of attempts
		#重复直到成功或者超出尝试的次数
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = pick_arm.pick(target_id, grasps)
            rospy.sleep(0.2)
        # If the pick was successful, attempt the place operation   
		#如果抓取成功，尝试放置操作
        result = None
        n_attempts = 2
        #if result == MoveItErrorCodes.SUCCESS:
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:      
            # Generate valid place poses
            #places = self.make_places(place_pose)
            n_attempts += 1
            print("-------------------")
            print(place_pose)
            pick_arm.set_start_state_to_current_state()
            pick_arm.set_pose_target(place_pose)
            # Repeat until we succeed or run out of attempts
            #while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
            #    n_attempts += 1
            #    rospy.loginfo("Place attempt: " +  str(n_attempts))
            #    for place in places:
            #        result = pick_arm.place(target_id, place)
            #        if result == MoveItErrorCodes.SUCCESS:
            #            break
            #    rospy.sleep(0.2)
            result = pick_arm.go(wait=True)
            rospy.logerr("pick_arm.go： " + str(result))
            open_client(500)
            rospy.sleep(0.2)
            if result != MoveItErrorCodes.SUCCESS:
                rospy.logerr("Place operation failed after " + str(n_attempts) + " attempts.")
        #else:
        #    rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
                
        # Return the arm to the "home" pose stored in the SRDF file
		#将机械臂返回到SRDF中的“home”姿态
        open_client(500)
        pick_arm.set_named_target(ARM_HOME_POSE)
        pick_arm.go()
        
        # Open the gripper to the neutral position
        # pick_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        # pick_gripper.go()
       
        rospy.sleep(1)

        # Shut down MoveIt cleanly
        # moveit_commander.roscpp_shutdown()
        
        # Exit the script
        # moveit_commander.os._exit(0)
        
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self,joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()
        
        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES
     
        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()
        
        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions
        
        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT
        
        tp.time_from_start = rospy.Duration(1.0)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t
    
    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()
        
        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME
        
        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()
        
        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)

        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, 0, 1.7])

        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
    
        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.5, 0.4, 0.6]
        
        # Yaw angles to try
        yaw_vals = [0]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            for p in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, p, y)
                
                # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                
                # Set and id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))
                
                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects
                
                # Don't restrict contact force
                g.max_contact_force = 0
                
                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(p)
                
                # Append the grasp to the list
                grasps.append(deepcopy(g))
                
        # Return the list
        return grasps
    
    # Generate a list of possible place poses
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()
        
        # Start with the input place pose
        place = init_pose
        
        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        pitch_vals = [0]
        
        # A list of yaw angles to try
        yaw_vals = [0]

        # A list to hold the places
        places = []

        places.append(deepcopy(init_pose))
        
        # Generate a place pose for each angle and translation
        for y in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.pose.position.x = init_pose.pose.position.x + x
                        place.pose.position.y = init_pose.pose.position.y + y
                        
                        # Create a quaternion from the Euler angles
                        #q = quaternion_from_euler(0, p, y)
                        
                        # Set the place pose orientation accordingly
                        #place.pose.orientation.x = q[0]
                        #place.pose.orientation.y = q[1]
                        #place.pose.orientation.z = q[2]
                        #place.pose.orientation.w = q[3]
                        
                        # Append this place pose to the list
                        places.append(deepcopy(place))
        
        # Return the list
        return places
    
    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

# 处理抓取过程
def handle_pick_and_place(req):
    t= threading.Thread(target=MoveItDemo,args=(req.pickPos,req.placePos))
    t.setDaemon(True)
    t.start()
    return pickPlaceResponse()

# 创建抓取与放置 服务端 其中pickPlace是保存数据格式的文件
def pick_and_place_server():
    rospy.init_node('pick_and_place_server')
    s = rospy.Service('pick_and_place', pickPlace, handle_pick_and_place)
    print "Ready to pick and place."
    rospy.spin()

if __name__ == "__main__":
    global open_client
    rospy.wait_for_service('gripper_open')
    open_client = rospy.ServiceProxy('gripper_open', open_srv)
    pick_and_place_server()

    
