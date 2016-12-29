#! /usr/bin/env python

import zbar
import cv2
import Image
import numpy as np

# create the image reader
scanner = zbar.ImageScanner()

# configure the reader
scanner.parse_config("enable")

# the control variable for the following loop
key = ord("0")

# the loop to read a QRcode 
while (key != ord('q')):
	cv2_im = cv2.imread("~/zbar-0.10/code.jpg")
	# convert the image from color to gray
	gray =cv2.cvtColor(cv2_im,cv2.COLOR_BGR2GRAY)
	# transfer into the python PIL image
	pil_im = Image.fromarray(gray)		
	width,height = pil_im.size
	# get the raw image
	raw = pil_im.tostring()
	# zip the image data into the format which can be read by zbar
	image = zbar.Image(width,height,'Y800',raw)
	
	# scan the QRcode
	scanner.scan(image)
	
	# show the date in the QRcode
	for symbol in image:
		print "decoded",symbol.type,'symbol','"%s"'%symbol.data
	
	
	key = cv2.waitKey(33)&0xFF
	del(image)
