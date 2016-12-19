#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class OutandBack():
	def __init__(self):
		rospy.init_node("out_and_back",anonymous=False)
		rospy.on_shutdown(self.shutdown)
		
		self.cmd_vel = rospy.Publisher("/cmd_vel",Twist,queue_size = 5)
		rate = 50
		r = rospy.Rate(rate)

		linear_speed = 0.2
		linear_distance = 1.0
		linear_duration = linear_distance/linear_speed

		angular_speed = 1.0
		angular_angle = pi
		angular_duration = angular_angle / angular_speed
	
		for i in range(2):
			move_cmd = Twist()
			move_cmd.linear.x = linear_speed

			ticks = int(linear_duration*rate)
			for t in range(ticks):
				self.cmd_vel.publish(move_cmd)
				r.sleep()
			move_cmd = Twist()
			self.cmd_vel.publish(move_cmd)
			rospy.sleep(1)

			move_cmd.angular.z = angular_speed
			ticks = int(angular_duration*rate)
			
			for t in range(ticks):
						
				self.cmd_vel.publish(move_cmd)
				r.sleep()
			move_cmd = Twist()
			self.cmd_vel.publish(move_cmd)
			rospy.sleep(1)
 		self.cmd_vel.publish(Twist())
	
	def shutdown(self):
		rospy.loginfo("stopping the robot...")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ = "__main__":
	try:
		OutandBakc()
	except:
		rospy.loginfo("out and back node terminated.")
