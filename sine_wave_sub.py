#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

def callback(msg):
	rospy.loginfo(msg.data)
rospy.init_node('sine_wave_sub')
sub = rospy.Subscriber('sine_wave',Float64,callback)
rospy.spin()

