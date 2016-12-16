#!/usr/bin/env python

import rospy 
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "T heard %s",data.data)



def listener():
	# in ros nodes are uniquely named. if two nodes with the same node are
	# launched,the previous one is kicked off. The anonymous =True flag means that 
	# rospy will choose a unique name for oyr 'listener' node so that so that multiple listenners	
	# can run simutaneously
	rospy.init_node("listener",anonymous=True)
	rospy.Subscriber("chatter",String,callback)
	
	# spin() simply keeps python from exiting until thie node is stopped
	rospy.spin()

if __name__ = "__main__":
	listener()
