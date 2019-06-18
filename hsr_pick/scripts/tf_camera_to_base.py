#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from object_recognition_msgs.msg import RecognizedObjectArray
import turtlesim.srv
from geometry_msgs.msg import PoseStamped

coke_can_pose = PoseStamped()
coke_can_pose.header.seq = 1
coke_can_pose.header.frame_id = "kinect2_rgb_optical_frame"

def callback(data):
    global coke_can_pose
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.objects[0].pose.pose)
    coke_can_pose.pose = data.objects[0].pose.pose.pose	

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    rospy.Subscriber("recognized_object_array", RecognizedObjectArray, callback)
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            base_link_pose = listener.transformPose('base_link', coke_can_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rospy.loginfo(rospy.get_caller_id() + "I heard %s", base_link_pose)
        rate.sleep()
