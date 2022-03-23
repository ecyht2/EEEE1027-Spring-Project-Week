#!/usr/bin/env python
from picamera.array import PiRGBArray
import time
import cv2
import picamera
import numpy as np

# Initialize camera
camera = picamera.PiCamera()
camera.resolution = (192,112)
camera.framerate = 20
rawCapture = PiRGBArray(camera,size=(192,112))
time.sleep(0.1)

# Loop over all frames captured by camera indefinitely
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # Display camera input
        img = frame.array
        # Flipping the image
        img = cv2.flip(img, 0)
        img = cv2.flip(img, +1)

        # converting image into grayscale image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # setting threshold of gray image
        _, threshold = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY)

        # using a findContours() function
        contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        i = 0

        # list for storing names of shapes
        for contour in contours:
                # here we are ignoring first counter because
                # findcontour function detects whole image as shape
                if i == 0:
                        i = 1
                        continue

                # cv2.approxPloyDP() function to approximate the shape
                approx = cv2.approxPolyDP(
                        contour, 0.01 * cv2.arcLength(contour, True), True)

                # using drawContours() function
                cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

                # finding center point of shape
                M = cv2.moments(contour)
                x = 0
                y = 0
                if M['m00'] != 0.0:
                        x = int(M['m10']/M['m00'])
                        y = int(M['m01']/M['m00'])

                # putting shape name at center of each shape
                if len(approx) == 3:
                        cv2.putText(img, 'Triangle', (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                elif len(approx) == 4:
                        cv2.putText(img, 'Quadrilateral', (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                elif len(approx) == 5:
                        cv2.putText(img, 'Pentagon', (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                elif len(approx) == 6:
                        cv2.putText(img, 'Hexagon', (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                else:
                        cv2.putText(img, 'circle', (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Create key to break for loop
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
                break

        cv2.imshow('image', img)
        cv2.imshow('threshold', threshold)
        rawCapture.truncate(0)
