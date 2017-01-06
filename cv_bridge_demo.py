#!/ usr/bin/env python

import rospy
import cv2
import sys

import cv2.cv as cv
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge,CvBridgeError
	
import numpy as np


class cvBridgeDemo():
	def __init__(self):
		self.node_name = 'cv_bridge_demo'
		rospy.init_node(self.node_name)
		#what we do during shutdown
		rospy.on_shutdown(self.cleanup)
		
		#create the opencv display window for the rgb image
		self.cv_window_name = self.node_name
		cv.NamedWindow(self.cv_window_name,cv.CV_WINDOW_NORMAL)
		cv.MoveWindow(self.cv_window_name,25,75)
		
		# and one for the depth image
		cv.NamedWindow("depth image",cv.CV_WINDOW_NORMAL)
		cv.MoveWindow('depth image',25,300)
		
		# create the cv_bridge object
		self.bridge = CvBridge()
		
		# subscribe to the camera image and depth topics and set
		# the appropriate callbacks
		self.image_sub = rospy.Subscriber('/camera/rgb/image_color',Image,self.image_callback)
		self.depth_sub = rospy.Subcriber('/camera/depth_registered/image_rect',Image,self.depth_callback)
		
		rospy.loginfo("waiting for the image topics...")
		

		
		
	def image_callback(self,ros_image):		
		# use the cv_bridge) to convert the ros image into opencv format
		try:
			frame = self.bridge.imgmsg_to_cv2(ros_image,'bgr8')
		except CvBridgeError,e:
			print e
		# convert the image into a numpy array since most cv2 functions require numpy array
		frame = np.array(frame,dtype=np.uint8)
		# process the image using process_image() function
		display_image = self.process_image(frame)
		# display the image
		cv2.imshow(self.node_name,display_image)
		
		# process any keyborad commands
		self.keystroke = cv.WaitKey(5)
		if 32<= self.keystroke and self.keystroke <128:
			cc = chr(self.keystroke).lower()
			if cc == 'q':
				rospy.signal_shutdown('User hit q key to exit')
		
	
	def depth_callback(self,ros_image):
		# use cv_bridge() to convert the ros image into opencv format
		try:
			depth_image = self.bridge.imgmsg_to_cv2(ros_image,'passthrough')
		except CvBridgeError,e:
			print e
		# convert the depth image to a numpy array
		depth_array = np.array(depth_iamge,dtype=np.float32)
		
		# normalize the dapth image to fall between 0 and 1
		cv2.normalize(depth_array,depth_array,0,1,cv2.NORM_MINMAX)
		
		# process the depth image
		depth_display_image = self.procee_depth_image(depth_array)
		# display teh result
		cv2.imshow('depth image',depth_display_image)
		
	
	def process_image(self,frame):
		# convert to grayscale
		grey = cv2.cvtColor(frame,cv.CV_BGR2GRAY)
		# Blur the image
		grey = cv2.blur(grey,(7,7))
		
		# compute edges using the canny edge filter
		edges = cv2.Canny(grey,15.0,30.0)
		return edges
	
		
	def process_depth_image(self,frame):
		# just return the raw image for this dmeo
		return frame
		
	def cleanup(self):
		print 'shutting down vision node'
		cv2.destroyAllWindows()
	
	
	
def main(args):
	try:
		cvBridgeDemo()
		rospy.spin()
	except KeyboardInterrrupt:
		print "shutting down vision node"
		cv.DestroyAllWindows()
	

if __name__ = '__main__':
	main(sys.argv)

	
