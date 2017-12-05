#!/usr/bin/env python
from geometry_msgs.msg import Twist
from ViconTrackerPoseHandler import ViconTrackerPoseHandler
import rospy
import math

def run():
	rospy.init_node("test", anonymous=True)

	vicon = ViconTrackerPoseHandler(None, None, "",51040, "helmet")

	# for i in range(0,4):
	# 	rospy.sleep(2.5)
	# 	print vicon.getPose()
	# 	rospy.loginfo(vicon.getPose())
	vel_msg = Twist()
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	rospy.sleep(1)
	#face towards target
	# vel_msg.angular.z = math.radians(-45) + 0.02
	# vel_msg.angular.z = math.radians(90) - 0.04
	vel_msg.angular.z = math.radians(180) + 0.13
	# vel_msg.angular.z = math.radians(-135) + 0.1
	vel_msg.linear.x =  0
	pub.publish(vel_msg)

	rospy.sleep(1.11)
	# angle = round(math.degrees(vel_msg.angular.z)/45)*45
	# if angle == 45 or angle == -45:
	# 	rospy.sleep(1.17)
	# elif angle == 90 or -90:
	# 	rospy.sleep(1.11)
	# else:
	# 	rospy.sleep(1)

	#stop
	vel_msg.angular.z = 0
	vel_msg.linear.x =  0
	pub.publish(vel_msg)
	rospy.sleep(1)
	rospy.loginfo("done")
if __name__ == "__main__":
	run()


