#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState

class ImpedenceContorl:
	
	def __init__(self):
		#self.robot = robot
		self.sub = rospy.Subscriber("joint_states", JointState, self.err_callback)
		self.update_timer = rospy.Timer(rospy.Duration(1), self.__update) 

	def err_callback(self, data):
		print("in callback", data.position)

	def start(self):
		print('start')
	
	def __update(self, event):
		print("hello")

if __name__ == '__main__':
	rospy.init_node('ur_test', anonymous=True)
	imp = ImpedenceContorl()
	imp.start()
	#rospy.spin()
	i = 1
	while True:
		i += 1
