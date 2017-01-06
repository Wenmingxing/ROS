#! /usr/bin/env python

import rospy
import cv2
import cv2.cv as cv
from rbx1_vision.ros2opencv2 importROS2OpenCV2

class FaceDetector(ROS2OpenCV2):
	def __init__(self,node_name):
		super(FaceDetector,self).__init__(node_name)
		# get the paths to the cascade XML files for the haar detectors.
		#these are set in the launch file.
		cascade_1 = rospy.get_param("~cascade_1",'')
		cascade_2 = rospy.get_param("~cascade_2",'')
		cascade_3 = rospy.get_param("~cascade_3",'')
	
		#initialize the haar detectors using the cascade files
		self.cascade_1 = cv2.CascadeClassifier(cascade_1)
		self.cascade_2 = cv2.CascadeClassifier(cascade_2)
		self.cascade_3 = cv2.CascadeClassifier(cascade_3)
	
		# set cascade parameters that ted to work well for faces
		# can be overridden in launch file
		self.haar_minSize = rospy.get_param('~haar_minSize',(20,20))
		self.haar_maxSize = rospy.get_param("~haar_maxSize",(150,150))
		self.haar_scaleFactor = rospy.get_param('~haar_scaleFactor',1.3)
		self.haar_minNeighbors = rospy.get_param('~haar_minNeighbors',1)
		self.haar_flags = rospy.get_param('~haar_flag',cv.CV_HAAR_DO_CANNY_PRUNING)
		#Store all parameters together for passing to the detector
		self.haar_params = dict(minSize = self.haar_minSize,
					maxSize = self.haar_maxSIze,
					scaleFactor = self.scaleFactor,
					minNeighbors = self.haar_minNeighbors,
					flags = self.haar_flags)
		
		# do we should text on the display
		self.show_text = rospy.get_param('~show_text',True)

		# Initialize the detection bos
		self.detect_box = None
		
		# Track the number of hits and misses
		self.hits = 0
		self.misses = 0
		self.hit_rate = 0


	def process_image(self,cv_image):
		# create a grayscale version of the image
		grey = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
				

		#Equalize the histogram to reduce lighting effects
		grey = cv2.equlizeHist(grey)

		# Attempt to detect a face
		self.detect_box = self.detect_face(grey)
		if self.detect_box is not None:
			self.hits +=1
		else:
			self.misses +=1
				
		# keep tabs on the hit rate so far
		self.hit_rate = float(self.hits)/(self.hits+self.misses)
		
		return cv_image
		
		
	def detect_face(self,input_image):
		# first check one of the frontal templates
		if self.cascade_1:
			faces = self.cascade_1.detectMultiScale(input_image,**self.haar_params)

		# if that fails, check the profile template
		if len(faces) == 0 and self.cascade_3:
			faces = self.cascade_3.detectMultiScale(input_image,**self.haar_params)
		# if that also fails, check a the other frontal template
		if len(faces)==0 and self.cascade_2:
			faces = self.cascade_2.detectMultiScale(input_image,**self.haar_params)
		# the faces variables holds a list of face boxes.
		# if one or more faces are detected ,return the first one.
		if len(faces)>0:
			face_box = faces[0]
		else:
			# if no faces detected, print the ' lost face ' message on the screen
			if self.show_text:
				font_face = cv2.FONT_HERSHEY_SIMPLEX
				font_scale = 0.5
				cv2.putText(self.marker_image,'LOST FACE!',(int(self.frame_size[0]*0.65),int(self.frame_size[1]*0.9)),font_face,font_scale,cv.RGB(255,50,50))
			face_box = None
		# display the hit rate so far
		if self.show_text:
			font_face = cv2.FONT_HERSHEY_SIMPLEX
			font_scale = 0.5
			cv.putText(self.marker_image,'hit rate:'+str(trunc(self.hit_rate,2)),(20,int(self.frame_size[1]*0.9)),font_face,font_scale,cv.RGB(255,255,0))

		return face_box
	
def trunc(f,n):
	slen = len("%.*f"%(n,f))
		
	return float(str(f)[:slen])

if __name__ == '__main__':
	try:
		node_name = 'face_detector'
		FaceDetector(node_name)
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down face detector node."
		cv2.destroyAllWindows()
