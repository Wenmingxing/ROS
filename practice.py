#!/usr/bin/env python


import rospy 
import actionlib 
from actionlib_msgs import *
from geometry_msgs.msg import Pose,PoseWithCovarianceStamped,Point,Quaternion,Twist
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from random import sample
from math import pow,sqrt

class NavTest():
	def __init__(self):
		rospy.init_node("nav_test",anonymous=True)
		rospy.on_shutdown(self.shutdown)
		# How long in seconds should the robot pause at each location?
		self.rest_time = rospy.get_param("~rest_time",10)
		# are we running in the fake simulator?
		self.fake_test = rospy.get_param("~fake_test",False)
		# goal state return values
		godal_state  = ["PENDING","ACTIVE","PREEMPTED","SUCCEEDED","ABORED","REJECTED","PREEMPTING","RECALLING"
                "RECALLED","LOST"]
		"""set up the goal locations. Poses are defined in the map frame.
		   an easy way to find the pose coordinates is to point_and _click
		   nav goas in rviz when running in the simulator. Pose coordinates are the displayed
		   in the terminal that was used to launch rviz"""
		locations = dict()
		locations["hall_foyer"]=Pose(Point(0.643,4.720,0.000),Quaternion(0.000,0.000,0.223,0.975))
		locations["hall_kitchen"]=Pose(Point(-1.994,4.382,0.000),Quaternion(0.000,0.000,-0.670,0.743))
		locations["hall_bedroom"]=Pose(Point(-3.719,4.401,0.000),Quaternion(0.000,0.000,0.733,0.680))
		locations["living_room_1"]=Pose(Point(0.720,2.229,0.000),Quaternion(0.000,0.000,0.786,0.000))		
		locations["living_room_2"]=Pose(Point(1.471,1.007,0.000),Quaternion(0.000,0.000,0.480,0.877))
		locations["dining_room_1"]=Pose(Point(-0.861,0.019,0.000),Quaternion(0.000,0.000,0.892,-0.451))
		
		#publisher to manually control the robot(e.g to stop it)
		self.cmd_vel_pub=rospy.Publisher("cmd_vel",Twist,queue_size=5)
		# subscribe to te move_base action server
		self.move_base = actionlib.SimpleActionClient("Move_base",MoveBaseAction)
		rospy.loginfo("waiting for move_base action server...")
		# wait 60 seconds for the action server to become available
		self.move_base.wait_for_server(rospy.Duration(60))
		rospy.loginfo("Connected to move base server")
		# a variable to hold the initial pose of the robot to be set by the user in tviz
		initial_pose=PoseWithCovarianceStamped()
		
		# variable to keep track of success rate,running time,and distance traveled
		n_locations = len(locations)
		n_goals = 0
		n_successes = 0
		i = n_locations
		distance_traveled = 0
		start_time = rospy.Time.now()
		running_time = 0
		location = ""
		last_location = ""
		# get the initial pose from the user
		rospy.loginfo("click on the map in rviz to set the initial pose...")
		rospy.wait_for_message("initialpose",PoseWithCovarianceStamped)
		self.last_location = Pose()
		rospy.Subscriber("initialpose",PoseWithCovarianceStamped,self.update_initial_pose)
		#make sure we have the initial pose
		while initial_pose.header.stamp == "":
			rospy.sleep()
		rospy.loginfo("starting navagation test")
		#begin the main loop and run through a sequence of locations
		while not rospy.is_shutdown():
			# if we've gone through the current sequence,
			# start with a new random sequence
			if i == n_locations:
				i = 0
				sequence = sample(locations,n_locations)
				#skip over first location if it is the same as the last location
			if sequence[0] == last_location:
				i = 1

		# get the next location in the current sequence
		location = sequence[i]
		# keep track of the diatance traveled.
		# use updated initial pose if available
		if initial_pose.header.stamp=="":
			distance = sqrt(pow(locations[location].position.x-locations[last_location].position.x,2)+
			pow(locations[location].position.y-locations[last_location].position.y,2))

		else:
			rospy.loginfo("updating current pose.")
			distance = sqrt(pow(locations[location].position.x-initial_pose.pose.pose.position.x,2)+pow(
			locations[location].position.y-initial_pose.pose.pose.position.y,2))
			initial_pose.header.stamp = ""
		# store the last location for the distance calculations
		last location = location
		#increment the counters
		i +=1
		n_goals +=1
		# set up the next goal location
		self.goal = MoveBaseGoal()
		self.goal.target_pose.pose = locations[location]
		self.goal.target_pose.header.frame_if = "map"
		self.goal.target_pose.header.stamp = rospy.Time.now()
		# let the user know where the robot is going next
   		rospy.loginfo("going to :"+str(location))
		# start the robot toward the next location
		self.move_base.send_goal(self.goal)
		# allow 5 minutes to get there
 		finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))
		# check for the success or failure
		if not finished_within_time:
			self.move_base.cancel_goal()
			rospy.loginfo("Timed out achieving goal")
		else:
			state = self.move.base.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.loginfo("Goal succeeded!")
				n_succeees += 1
				distance_traveled +=distance
			else:
				rospy.loginfo("goal failed with error code:"+str(goal_states[state]))
		# how long have we been running
		running_time = rospy.Time.now()-start_time
		running_time = running_time.secs/60
		# print a summary success/failure,distance traveled and time elapsed
		rospy.loginfo("success so far:"+str(n_successes)+"/"+str(n_goals)+"="+str(100*n_successes/n_goals)+"%")
		rospy.loginfo("Running time:"+str(trunc(running_time,1))+"min Distance"+str(trunc(distance_traveled,1))+"m")
		rospy.sleep(self.rest_time)
    
	def update_initial_pose(self,initial_pose):
		self.initial_pose  = initial_pose
       	def shutdown(self):
		rospy.loginfo("stopping the robot...")
		self.move_base.cancel_goal()
		rospy.sleep(2)
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)
	

	def trunc(f,n):
		# truncates/pads a float f to n decimal places without rounding
		slen = len("%.*f"%(n,f))
		return float(st(f)[:slen])

	if __name__ == '__main__':
		try:
			NavTest()
			rospy.spin()
		except rospy.ROSInterruptException:
			rospy.loginfo("AMCL navigation test finished.")
