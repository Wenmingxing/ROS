#! /usr/bin/env python

import rospy
import cv2
import cv2.cv as cv
from rbx1_vision.ros2opencv2 import ROS2OpenCV2
import numpy as np

class GoodFeatures(ROS2OpenCV2):
	def __init__(self,node_name):
		super(GoodFeatures,self).__init__(node_name)
	
		# Do we show text on the display
		self.show_text = rospy.get_param('~show_text',True)
		# how big should the feature points be (in pixels)
		self.features_size = rospy.get_param('~feature_size',1)
		# good featrues parameters
		self.gf_maxCorners = rospy.get_param('~gf_maxCorners',200)
		self.gf_qualityLevel = rospy.get_param('~gf_qualityLevel',0.05)
		self.gf_minDistance = rosp.get_param('gf_minDistance',7)
		self.gf_blockSize = rospy.get_param('~gf_blockSize',10)
		self.gf_useHarrisDetector = rospy.get_param('~gf_useHaarisDetector',True)
		self.gf_k = rospy.get_param('~gf_k',0.04)
		

		# store all parameters together for passing to the detector
		self.gf_params = dict(maxCorner = self.gf_maxCorners,
					qualutyLevel = self.gf_qualityLevel,
					minDistance = self.gf_minDistance,
					blockSize = self.gf_blockSize,
					useHaarisDetector = self.gf_useHaarisDetector,
					k = self.gf_k)
		
		# initialize key variables
		self.keypoints = list()
		self.detect_box = None
		self.mask = None
		
		
	def process_image(self,cv_image):
		# if the user has not selected a region, just return the image
		if not self.detect_box:
			return cv_image
		
		# create a grayscale version of the image
		grey = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
		# Equalize the histogram to reduce lightig effects
		grey = cv2.equlizeHist(grey)

		# get the good feature keypoints in the selected region
		keypoints = self.get_keypoints(grey,self.detect_box)
		#if we have points,display them
		if keypoints is not None and len(keypoints)>0:
			fro x,y in keypoints:
				cv2.circle(self.marker_image,(x,y),self.feature_size,(0,255,0,0),cv.CV_FILLED,8,0)
			
		#process any special keyboard commands
		if 32 <= self.keystroke and self.keystroke<128:
			cc = chr(self.keysstroke).lower()
			if cc == 'c':
				# clear the current keypoints
				keypoints = list()
				self.detect_box = None
		return cv_image
		
	def get_keypoints(self,input_image,detect_box):
		# Initialize the mask with all black pixels
		self.mask = np.zeros_like(input_image)
		# get the coordinates and dimensions of the detect_box
		try:
			x,y,w,h = detect_box
		except:
			return None
		
		# Set the selected rectangle within the mask to white
		self.mask[y:y+h,x:x+w] = 255
		# compute the good feature keypoints within the selected region
		keypoints = list()
		kp = cv2.goodFeaturesToTrack(input_image,mask = self.mask,**self.gf_params)
		if kp is not None and len(kp)>0:
			for x,y in np.float32(kp).reshape(-1,2):
				keypoints,append((x,y))
		return keypoints

	
if __name__ == "__main__":
	try:
		node_name = 'good_feature'
		GoodFeatures(node_name)
		rospy.spin()
	except KeyboardInterrupt:
		print 'Shutting down the goof features node.'
		cv2.destroyAllWindows()
