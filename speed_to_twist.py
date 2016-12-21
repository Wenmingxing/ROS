#!/usr/bin/env python

import roslib
roslib.load_manifest("beginner_tutorials")

import rospy

from beginner_tutorials.msg import Num,carodom
from geometry_msgs.msg import Twist

import serial_listenning as COM_ctr
import glob
from math import sqrt,atan2,pow

class bluetooth_cmd():
	def __init__(self):
		rospy.init_node("robot_driver",anonymous=True)
	def callback(self,msg):
		cmd_twist_rotation = msg.angular.z
		cmd_twist_x = msg.linear.x*10.0
		cmd_twist_y = msg.linear.y*10.0
		
		wheelspeed = self.odom_to_speed(cmd_twist_x,cmd_twist_y,cmd_twist_rotation)
		print "msg:",msg
		print wheelspeed
		
		self.blue_tooth_send([wheelspeed[0],self.speed_kp,self.speed.ki,wheelspeed[1]])
	
	def odom_to_speed(self,cmd_twist_x =0,cmd_twist_y=0,cmd_twist_rotation=0):
		cent_speed = sqrt(pow(cmd_twsit_x,2)+pow(cmd_twsit_y,2))
		yawrate2 = self.yawrate_to_speed(cmd_twist_rotation)
		
		lwheelspeed = cent_speed - yawrate2/2
		rwheelspeed = cent_speed + yawrate2/2

		return lwheelspeed,rwheelspeed

	def yawrate_to_speed(self,yawrate):
		if yawrate > 0:
			theta_to_speed = 0.0077
		else:
			theta_to_speed = 0.0076
		
		x = (yawrate * 0.02)/theta_to_speed
		return x

		
	def talker(self):
		self.rec_data = COM_ctr.SerialData(datalen = 2)
		allport = glob.glob("/dev/ttyU*")
		port = allport[0]
		baud = 115200
		openflag = self.rec_data.ope_com(port,baud)
		
		rospy.Subscriber("/cmd_vel",Twist,self.callback)
		pub = rospy.Publisher("car_speed",carOdom)
		pub_wheel = rospy.Publisher("wheel_speed",Num)
		
		r = rospy.Rate(500)
		lwheelpwm = 0
		suml = 0
		sumr = 0
		
		while not rospy is_shutdown():
			all_data = []
			if self.rec_data.com_isopen():
				all_data = self.rec_data.next()
			if all_data != []:
				wheelspeed = Num()
				car_speed = carOdom()
				leftspeed = all_data[0][0]
				rightspeed = all_data[1][0]
				wheelspeed.leftspeed = leftspeed
				wheelspeed.rightspeed = rightspeed
			
				results = self.speed_to_odom(leftspeed,rightspeed)
				car_speed.x = results[0]
				car_speed.y = results[1]
				car_speed.vth = results[2]

				pub.publish(car_speed)	
				pub_wheel.publish(wheelspeed)
			r.sleep()
		if openflag:
			self.rec_data.close_listen_com()

	def speed_to_odom(self,lspeed=0,rspeed=0):
		delta_speed = rspeed - lspeed
		if delta_speed < 0:
			theta_to_speed = 0.0077
		else:
			theta_to_speed = 0.0076
		v_th = delta_speed * theta_to_speed/0.02
		v_x = (rspeed+lspeed)/10.0/2.0
		v_y = 0.0
		
		return v_x,v_y,v_th
	
	def blue_tooth_send(self,data=[],head="HY")
		if data !=[] and self.rec_data.com_isopen():
			self.rec_data.send_data(data,head)
			

	if __name__ = "__main__":
		try:
			car_cmd = bluetooth_cmd()
			car_cmd.talker()
		except rospy.ROSInterruptException:
			pass			

