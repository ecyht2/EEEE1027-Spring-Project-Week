#!/usr/bin/env python

# Gregory Mueth
# 2635999
# EEL 4660 Robotics Systems
# Fall 2017

# line-follower.py

# A line following robot for Robotics Systems final project
# Coded using Python 2.7.13 and OpenCV 3.3 for a Raspberry Pi 3B running Raspbian
# Uses Raspberry Pi camera module v2

from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
import time
import cv2
import picamera
import numpy as np
import robot

# Initialize camera
camera = picamera.PiCamera()
camera.resolution = (192, 112)
camera.framerate = 20
rawCapture = PiRGBArray(camera,size=(192, 108))
time.sleep(0.1)

# setup GPIO pins
robot.setup()
car = robot.car

# Loop over all frames captured by camera indefinitely
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

	# Display camera input
	image = frame.array
	# Flipping the image
	image = cv2.flip(image, 0)
	image = cv2.flip(image, +1)
	#image = image[15:]

	# Create key to break for loop
	key = cv2.waitKey(1) & 0xFF

	# convert to grayscale, gaussian blur, and threshold
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray,(5,5),0)
	ret,thresh = cv2.threshold(blur, 30, 70, cv2.THRESH_BINARY_INV)

	# Erode to eliminate noise, Dilate to restore eroded parts of image
	mask = cv2.erode(thresh, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# Find all contours in frame
	contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	#cv2.imshow('mask', mask)


	# Find x-axis centroid of largest contour and cut power to appropriate motor
	# to recenter camera on centroid.
	# This control algorithm was written referencing guide:
	# Author: Einsteinium Studios
	# Availability: http://einsteiniumstudios.com/beaglebone-opencv-line-following-robot.html
	if len(contours) > 0:
		# Find largest contour area and image moments
		c = max(contours, key = cv2.contourArea)
		M = cv2.moments(c)

		# Find x-axis centroid using image moments
		cx = int(M['m10']/M['m00'])

		if cx >= 150:
			cv2.putText(image, 'right', (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
			car.turn_right(35, 1)
			print("right")

		if cx < 150 and cx > 40:
			cv2.putText(image, 'forward', (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
			car.forward(30)
			print("forward")

		if cx <= 40:
			cv2.putText(image, 'left', (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
			car.turn_left(35, 1)
			print("left")
	else:
		car.stop ()
		print("stop")


		#cv2.putText(image, str(cx), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

	#cv2.imshow('image', image)
	if key == ord("q"):
            break

	rawCapture.truncate(0)

robot.exit()
