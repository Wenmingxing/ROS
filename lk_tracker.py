#! /usr/bin/env python

import rospy
import cv2
import cv2.cv as cv
import numpy as np
from rbx1_vision.good_features import GoodFeatures


class LKTracker(GoodFeatures):
	def __init_(self,node_name):
		super(LKTracker,self).__init__(node_name)
		self.show_text = rospy.get_param('~show_text',True)
		self.feature_size = rospy.get_param('~feature_size',1)
		
		# Lk parameters
		self.lk_winSize = rospy.get_param('~lk_winSize',(10,10))
		self.lk_maxLevel = rospy.get_param('~lk_maxLevel',2)
		self.lk_criteria = rospy.get_param('~lk_criteria',(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,20,0.01))
		self.lk_derivLambda = rospy.get_param('~lk_derivLambda',0.1)
		self.lk_params = dict(winSize=self.lk_winSize,
				maxLevel = self.lk_maxLevel,
				criteria = self.lk_criteria,
				derivLambda = self.lk_derivLambda)
		self.detect_interval =1
		self.keypoints = list()
		
		self.detect_box = None
		self.track_box = None
		self.mask =  None
		self.grey = None
		self.prev_grey = None
	
		
	def process_image(self,cv_image):
		# if we don't yet have a detection box(draw by the user
		# with the mouse),keep waiting
		if self.detect_box is None:
			return cv_image
		
		# create a grayscale version of the image
		self.grey = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
		# Equalize a grey histogram to minimize lighting effects
		self.grey = cv2.equalizeHist(self.grey)
		
		# if we haven't yet started tracking, set the track box to the 
		# detect box and extract the keypoints within it
		if self.track_box is None or not self.is_rect_nonzero(self.track_box):
			self.track_bos = self.detect_box
			self.keypoints = list()
			self.keypoints = self.get_keypoints(self.grey,self.track_box)
			
		else:
			if self.prev_grey is None:
				self.prev_grey = self.grey
			# now that have keypoints, track them to the next frame
			# using optical flow
			self.track_box = self.track_keypoints(self.grey,self.prev_grey)
			
		# process any special keyboard commands for this module
		if 32<=self.keystroke and self.keystroke <128:
			cc == chr(self.keystroke).lower()
			if cc == 'c':
				# clear the current keypoints
				self.keypoints = list()
				self.track_box = None
				self.detect_box = None
				self.classifier_initialized = True
		self.prev_grey = self.grey
		return cv_image
	
		
	def track_keypoints(self,grey,prev_grey):
		try:
			# we are tracking points between the previous frame and the current frame
			img0,img1 = prev_grey,grey
		
			# reshape the current keypoints into a numpt array required by calcOpticalFlowPyrLK()
			p0 = np.float32([p for p in self.keypoints]).reshape(-1,1,2)
 			
			# calculate the optical flow from the previous frame
			# tp the current frame
			p1,st,err = cv2.calcOpticalFlowPyLK(img0,img1,p0,None,**self.lk.params)
			# do the reverse ca;culation : from the current frame to te previous one
			p0r,st,err = cv2.calcOpticalFlowPyrLK(imag1,imag0,p1,NOne,**self.lk_params)
			# compute the distance between corresponding points in the two flows
			d = abs(p0-p0r).reshape(-1,2).max(-1)
			# if the distance between pairs of points is <1 pixel,set 
			# a value in the 'good' array to True,Otherwise False
			good = d<1
			
			#Initialize a list to hold new keypoints
			new_keypoints = list()
			#cycle through all current and new keypoints and only keep 
 			#those that satisfy the 'good' condition above
			for (x,y),good_flag in zip(p1.reshape(-1,2),good):
				if not good_flag:
					continue
				new_keypoints.append((x,y))
				# Draw the keypoint on the image
				cv2.circle(self.marker_iamge,(x,y),self.feature_size,(0,255,0,0),cv.CV_FILLED,8,0)
 			# set the global keypoint list to the new list
			self.keypoints = new_keypoints
			
			# if we have >6 points,find the best ellipse around them 
			if len(self.keypoints)>6:
				self.keypoints_matrix = cv.CreateMat(1,len(self.keypoints),cv.CV_32sc2)
			i = 0
			for p in self.keypoints:
				cv.Set2D(self.keypoints_matrix,0,i,(int(p[0]),int(p[1])))
					
				i = i+1
				
			track_box = cv.FitEllipse2(self.keypoints_matrix)
		else:
			# otherwise, find the best fitting rectangle
			track_box = cv2.boundingRect(self.keypoints_matrix)
		except:
			track_box = None


		return track_box
		
			

if __name__ == '__main__':
	try:
		node_name = 'lk_tracker'
		LKTracker(node_name)
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down LK tracking node."
		cv2.detroyAllWindows()
				

	
