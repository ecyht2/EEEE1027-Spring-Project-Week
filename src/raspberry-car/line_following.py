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
from classes import PID, Colour
from math import atan

# Initialize camera
camera = picamera.PiCamera()
camera.resolution = (192, 112)
camera.framerate = 20
rawCapture = PiRGBArray(camera,size=(192, 108))
time.sleep(0.1)

# setup GPIO pins
car = robot.car

# Initialize PID values
K_P = 0.4
K_I = 0
K_D = 0.6
baseline = 96
basespeed = 30
pid = PID(baseline, K_P, K_I, K_D)
colours = {
#	"green": {
#		"class": Colour("green"),
#	},
	"blue": {
		"class": Colour("blue"),
	},
#	"yellow": {
#		"class": Colour("yellow"),
#	},
	"red": {
		"class": Colour("red"),
	}
}

def loop():
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

		threshold = [0, 0]
		with open("/home/pi/project/threshold.txt", "r") as f:
			threshold[0] = int(f.readline())
			threshold[1] = int(f.readline())

		# convert to grayscale, gaussian blur, and threshold
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(gray,(5,5),0)
		ret,thresh = cv2.threshold(blur, threshold[0], threshold[1], cv2.THRESH_BINARY_INV)

		# Erode to eliminate noise, Dilate to restore eroded parts of image
		mask = cv2.erode(thresh, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		for colour in colours.values():
			colour["class"].update_image(image)
			colour["contours"] = colour["class"].get_contours()

		# Find all contours in frame
		contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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

		for name, colour in colours.items():
			if len(colour["contours"]) > 0:
				c_colour = max(colour["contours"], key = cv2.contourArea)
				M_c = cv2.moments(c_colour)
				x = int(M_c['m10']/(M_c['m00'] + 1e-5))
				y = int(M_c['m01']/(M_c['m00'] + 1e-5))
				area = cv2.contourArea(c_colour)
				if area > 600:
					print("x: {}".format(x), "A: {}".format(area), "c: {}".format(name))
					cx = x

		# Updating PID
		pid.update(cx)
		PID = pid.get_PID()
		# Moving Car
		car.move_car(basespeed - PID, basespeed + PID)

		#cv2.imshow('threshold', thresh)
		#cv2.imshow('image', image)
		if key == ord("q"):
			break

		rawCapture.truncate(0)

if __name__ == '__main__':
	try:
		robot.setup()
		loop()
	finally:
		robot.exit()
