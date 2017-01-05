#! /usr/bin/env python

import math,time
import rospy
from std_msgs.msg import Float64

rospy.init_node('sine_wave_publisher',anonymous=True)

pub = rospy.Publisher('sine_wave',Float64,queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
	msg = Float64()
	msg.data = math.sin(4*time.time())
	pub.publish(msg)
	rate.sleep()

