#! /usr/bin/env python

from math import pi
from threading import Lock

from fake_sensor import FakeSensor

import rospy
import tf

from geometry_msgs.msg import Quaternion

def make_quaternion(angle):
	q = tf.transformations.quaternion_from_euler(0,0,angle)
	return Quaternion(*q)

def save_value(value):
	with lock:
		angle = value*2*pi/100.0
		

if __name__ == "__main__":
	lock = Lock()

	sensor = FakeSensor()
	sensor.register_callback(save_value)
	
	rospy.init_node("fake_sensor")
	pub = rospy.Publisher("angle",Quaternion,queue_size=10)
	
	angle = None
	rate = rospy.Rate(10)	
	while not rospy.is_shutdown():
		with lock:
			if angle:
				q = make_quaternion(angle)
				pub.publish(q)	
		rate.sleep()
	
