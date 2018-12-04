#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from pick_and_place.srv import *
from geometry_msgs.msg import PoseStamped, Pose
 
def pick_and_place_client(a, b):
    rospy.wait_for_service('pick_and_place')
    try:
        pick_and_place = rospy.ServiceProxy('pick_and_place', pickPlace)
        resp1 = pick_and_place(a, b)
        return resp1.isPickFinshed
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"

if __name__ == "__main__":
    a = PoseStamped()
    a.pose.position.x = 0.47
    a.pose.position.y = -0.4
    a.pose.position.z = 0.07
    a.pose.orientation.w = 1.0

    b = PoseStamped()
    b.pose.position.x = 0.43
    b.pose.position.y = -0.18
    b.pose.position.z = 0.07
    b.pose.orientation.w = 1.0

    pick_and_place_client(a, b)
    
    print "pick and place have been finished"
#    print "Requesting %s+%s"%(x, y)
#    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
