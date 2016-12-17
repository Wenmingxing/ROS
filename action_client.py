#!/usr/bin/env python
import roslib
roslib.load_manifest("my_pkg_name")

import rospy
import actionlib

from chores.msg import DoDishesAction,DoDishesGoal

if __name__ = "__main__":
	rospy.init_name("do_dishes_client",anonymous = True)
	client = actionlib.SimpleActionClient("do_dishes",DoDishesAction)
	client.wait_for_server()
	
	goal = DoDishesGoal()
	# fill in the goal here
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5.0))
