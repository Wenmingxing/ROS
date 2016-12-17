#!/usr/bin/env python
import roslib
roslib.load_manifest("my_pkg")

import rospy
import actionlib

from actionlib.msg import DoDishesAction



class DoDishesServer():
	def __init__(self):
		self.server = actionlib.SimpleActionServer("do_dishes",DoDishesAction,self.excute,False)
		self.server.start()
	def execute(self,goal):
		# DO lots of awesome groundbreaking root stuff here
		self.server.set_succeed()


if __name__ ="__main__":
	rospy.init_node("do_dishes_server")
	server = DoDishesServer()
	rospy.spin()
		

