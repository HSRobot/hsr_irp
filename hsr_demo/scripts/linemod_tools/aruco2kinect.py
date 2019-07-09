#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import cv2


if __name__ == '__main__':
    rospy.init_node('test')

    br = tf.TransformBroadcaster()
 

    rate = rospy.Rate(10.0)
    br.sendTransform((-0.1095, 0, 0.1095), (0,0,0,1), rospy.Time.now(), "origin", "/camera_marker")

    #listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/kinect2_rgb_optical_frame', '/camera_marker', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print(trans)

   cv2.projectPoints()
        rate.sleep()
