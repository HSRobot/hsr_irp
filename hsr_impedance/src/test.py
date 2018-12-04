#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

class ImpedenceContorl:


	def start(self):
		print("start---------------")
		self.update_timer = rospy.Timer(rospy.Duration(1), self.__update)

	def __update(self, event):
		print("start----ss-----------")

		#self.robot.send_servoj(999, target_post, 4 * self.RATE)


def main():
    rospy.init_node('fuck', disable_signals=True)
    imped = ImpedenceContorl()
    imped.start()
    rospy.spin()

if __name__ == '__main__': main()
