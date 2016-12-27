#! /usr/bin/env python

import rospy

import time
import actionlib
from luke.msg import TimerAction,TimerGoal,TimerResult,TimerFeedback

def feedback_cb(feedback):
	print("[feedback] Time elapsed: %f"%(feedback.time_elapsed.to_sec()))
	print("[feedback] Time remaining: %f"%(feedback.time_remaining.to_sec()))
	

rospy.init_node("time_action_client")
client = actionlib.SimpleActionClient("timer",TimerAction)
client.wait_for_server()
goal = TimerGoal()
goal.time_to_wait = rospy.Duration.from_sec(5)

client.send_goal(goal,feedback_cb=feedback_cb)

client.wait_for_result()

print("[result] state: %d"%(client.get_state()))
print("[result] status: %s"%(client.get_goal_status_text()))
print("[result] Time elapsed: %f"%(client.get_result().time_elapsed.to_sec()))
print("[result] updates send:%d"%(client.get_result().updates_sent))	

