#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist

class TestMove():

	def __init__(self):
		# 初始化ROS节点
		rospy.init_node("test_move", anonymous = False)

		# 设置程序退出时的回调函数
		rospy.on_shutdown(self.shutdown)

		# cmd_vel发布者的声明
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# 设置运动的X方向线速度
		linear_speed = 0.2

		# 设置运动的X方向线速度
		angular_speed = -0.2

		# 话题发布速率
		rate = 50

		# 构建休眠器，周期为rate
		r = rospy.Rate(rate)

		# 初始化运动消息
		move_cmd = Twist()

		# 修改消息中的线速度
		move_cmd.linear.x =  linear_speed

		# 修改消息中的角速度
		move_cmd.angular.z = angular_speed

		while not rospy.core.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			r.sleep()

		# 停止运动
		move_cmd = Twist()
		self.cmd_vel.publish(move_cmd)

		# logging
		rospy.loginfo("Robot move finish")

	def shutdown(self):
		rospy.loginfo("Stopping the robot...")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ == '__main__':
	TestMove()

