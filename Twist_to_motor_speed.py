#/usr/bin/env python

import roslib
roslib.load_manifest("beginner_tutorials")

import rospy
import tf.transformations
from geometry_msgs.msg import Twist


class twist_to_speed(object):
	def __init__(self):
		rospy.init_node("Twist_to_speed")
		rospy.Subscriber("/cmd_vel",Twist,self.callback)
                
	def callback(self,msg):
		cmd_twist_rotation = msg.angular.z
		cmd_twist_x = msg.linear.x
		cmd_twist_y = msg.linear.y
		
		# Translate the Twist message into motor expected speed
		wheelspeed = self.odom_to_speed(cmd_tsist_x,cmd_twist_y,cmd_twist_rotation)
		print("msg:",msg)
		print wheelspeed

		self.blue_tooth_send(wheelspeed[0],self.speed_kp,self.speed_ki,wheelspeed[1])

	def odom_to_speed(self,cmd_twist_x=0,cmd_twsit=0,cmd_twsit_rotation=0):
		cent_speed = cmd_twsit_x #the central speed
	
		yawrate2 = self.yawrate_to_speed(cmd_twist_rotation)
		lwheelspeed = cent_speed - yawrate2/2
		rwheelspeed = cent_speed + yawrate2/2
	
		return lwheelspeed,rwheelspeed

	def yawrate_to_speed(self,yawrate):
		if yawrate > 0:
			theta_to_speed = 0.0077 # the turning rate fpr right motor
		else:
			theta_to_speed = 0.0076		
		x = (yawrate * 0.02)/theta_to_speed


