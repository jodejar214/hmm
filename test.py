#!/usr/bin/env python
from geometry_msgs.msg import Twist
import rospy
import math

def run():
	rospy.init_node("test", anonymous=True)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	for i in range(0,3):
		vel = Twist()
		rospy.sleep(1)
		vel.linear.x = 1/3
		pub.publish(vel)
		rospy.sleep(1)
		vel.linear.x = 0
		pub.publish(vel)
		vel.angular.z = math.pi/2.
		rospy.sleep(1)
		pub.publish(vel)
		vel.angular.z = 0
		rospy.sleep(1)
		pub.publish(vel)
	rospy.loginfo("done")
if __name__ == "__main__":
	run()


