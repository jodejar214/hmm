#!/usr/bin/env python
from geometry_msgs.msg import Twist
from ViconTrackerPoseHandler import ViconTrackerPoseHandler
import rospy
import math

def run():
	rospy.init_node("test", anonymous=True)

	vicon = ViconTrackerPoseHandler(None, None, "",51023, "ScottsHead")

	for i in range(0,4):
		rospy.sleep(2.5)
		print vicon.getPose()
		rospy.loginfo(vicon.getPose())

	# pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	# for i in range(0,3):
	# 	vel = Twist()
	# 	rospy.sleep(1)
	# 	vel.linear.x = 0.25
	# 	pub.publish(vel)
	# 	rospy.sleep(1.4)
	# 	vel.linear.x = 0
	# 	vel.angular.z = math.radians(90)
	# 	pub.publish(vel)
	# 	rospy.sleep(1.1)
	# 	pub.publish(vel)
	# 	vel.angular.z = 0
	# 	pub.publish(vel)
	rospy.loginfo("done")
if __name__ == "__main__":
	run()


