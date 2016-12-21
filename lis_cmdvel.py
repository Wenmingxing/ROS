#!/usr/bin/env python

import roslib
roslib.load_manifest("beginner_tutorials")

import rospy
import tf.transformations
from geometry_msgs.msg import Twist


def callback(msg):
	rospy.loginfo("Received a /cmd_vel message!")
	rospy.loginfo("Linear components: [%f,%f,%f]"%(Twist.linear.x,Twist.linear.y,Twist.linear.z))
	rospy.loginfo("angular components:[%f,%f,%f]"%(Twist.angular.x,Twist.angular.y,Twist.angular.z))
	

	# Do velocity processing here
	# Use the kinematics of your robot to map linear and angular velocity into motor commands
	# v_1 = 
	# v_2 = 
	
	# Then set your wheel speeds(using wheel_left and wheel_right as examples)
	# wheel_left.set_speed(v_1)
	# wheel_right.set_speed(v_2)


def listener():
	rospy.init_node("cmd_vel_listener")
	rospy.Subscriber("/cmd_vel",Twist,callback)
	rospy.spin()

if __name__ = "__main__"：
	listener()
