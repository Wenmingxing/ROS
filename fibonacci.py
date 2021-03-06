#!usr/bin/env python

import rospy
import actionlib

import actionlib_tutorials.msg


class FibonacciAction(object):
	# create messages that are used to publish feedback and result
	_feeedback = actionlib_tutorials.msg.FibonacciFeedback()
	_result = actionlib_tutorials.msg.FibonacciResult()\
0
	def __init__(self,name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name,actionlib_turtorials.msg.FibonacciAction,execute_cb = self.execute_cb,auto_start = False)
		

	def execute_cb(self,goal):
		# helper variable
		r = rospy.Rate(1)
		success = True
		
		# append the seeds for the fibonacci sequence
	
		rospy.loginfo("%s:executing,creating fibonacci sequence of order %i with seeds %i,%i"%(self._action_name,goal.order,self._feedback.sequence[0],self._feedback.sequence[1]))

		for i in range(1,goal.order):
			# check that preempt has not been requested bt the client
			if self._as.is_preempt_requested():
				rospy.loginfo("%s:Preempt"%self._action_name)
				self._as.set_preempted()
				success = False
				break
			self._feedback.sequence.append(self._feedback.sequence[i]+self._feedback.sequence[i-1])
			# publish the feedback
			self._as.publish_feedback(self._feedback)
			# this step is not neccessary the squence is computed at 1 hz for demostration purpose.
			r.sleep()
		if success:
			self._result.sequence=self._feedback.sequence
			rospy.loginfo("%s:succeeded"%self._action_name)
			self._as.set_succeeded(self._result)


if __name__ ="__main__":
	rospy.init_node("fibonacci")
	server = FibonacciServer(rospy.get_name())
	rospy.spin()
