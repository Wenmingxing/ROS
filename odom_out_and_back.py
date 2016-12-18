#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point,Quaternion
import tf

from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians,copysign,sqrt,pow,pi

class outandback():
	def __init__(self):
		rospy.init_node("out_and_back",anonymous = False)

		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher("/cmd_vel",Twist,queue_size = 5)
		rate = 20
		
		r = rospy.Rate(rate)
		linear_speed = 0.2
		goal_distance = 1.0
		angular_speed = 1.0
		goal_tolerance = radians(2.5)
		goal_angle = pi

		self.tf_listener = tf.TransformListener()
		rospy.sleep(2)
		self.odom_frame = "/odom"
		
		try:
			self.tf_listener.waitForTransform(self.odom_frame,'/base_footprint',rospy.Time(),rospy.Duration(1.0))
			self.base_frame = "/base_footprint"
		except (tf.Exception,tf.ConnectivityException,tf.LookupException):
			try:
				self.tf_listener.waitFotTransform(self.odom_frame,"/base_link",rospy.Time(),rospy.Duration(1.0))
				self.base_frame = "/base_link"
			except (tf.Exception,tf.ConnectivityException,tf.LoopupException):
				rospy.loginfo("can not find transform between /odom and /base_link and /base_footprint")
				rospy.signal_shutdown("tf Exception")
		position = Point()

		for i in range(2):
			move_cmd = Twist()
			move_cmd.linear.x = linear_speed

			(position,rotation) = self.get_odom(0)
			x_start = position.x
			y_start = position.y
	
			distance = 0
		
			while distance < goal_distance and not rospy.is_shutdown():
				self.cmd_vel.publish(move_cmd)
				r.sleep()
				(position,rotation) = self.get_odom()
				distance = sqrt(pow((position.x-x_start),2)+pow((position.y-y_start),2))
			move_cmd = Twist()
			self.cmd_vel.publish(move_cmd)
			rospy.sleep(1)
			
			move_cmd.angular.z = angular_speed
	    		last_angle = rotation
			turn_angle = 0 
			while abs(turn_angle + angle_tolerance) < abs(angular_goal) and not rospuy.is_shutdown():
				self.cmd_vel.publish(move_cmd)
				r.sleep()
				(position,rotation) = self.get_odom()
				delta_angle = normalize_angle(rotation - last_angle)
				turn_angle += delta_angle
				last_angle = rotation
 				
			move_cmd = Twist()
			self.cmd_vel.publish(move_cmd)
			rospy.sleep(1)
		self.cmd_vel.publish(Twist())
	
	def get_odom(self):
		try:
			(trans,rot) = self.tf_listener.looupTransform(self.odom_frame,self.base_frame,rospy.Time(0))	
		except (tf.Exception,tf.ConectivityException,tf.lookupException):
			rospy.loginfo("TF exception")
			return
		return (Point(*trans),quat_to_angle(Quaternion(*rot)))
	
	def shutdown(self):
		rospy.loginfo("stopping the robot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)
if __name__ = "__main__":
	try:
		outandback()
	except:
		rospy.loginfo("out and back node terminated.")

