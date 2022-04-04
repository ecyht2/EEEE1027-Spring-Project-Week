#!/usr/bin/env python
from picamera.array import PiRGBArray
import time
import cv2
import picamera
import numpy as np
from color_trackbar import Color_Trackbar

# Initialize camera
camera = picamera.PiCamera()
camera.resolution = (192, 112)
camera.framerate = 20
rawCapture = PiRGBArray(camera,size=(192, 108))
time.sleep(0.1)

#frame = camera.capture(rawCapture, format="bgr", use_video_port=True)
#image = frame
trackbar = Color_Trackbar("blue")
trackbar.load_values()
trackbar.GUI()
# Loop over all frames captured by camera indefinitely
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # Display camera input
        image = frame.array
        # Flipping the image
        image = cv2.flip(image, 0)
        image = cv2.flip(image, +1)
        #image = image[15:]
        blur = cv2.GaussianBlur(image,(5,5),0)

        # Updating Tracbar
        trackbar.update_img(blur)
        # Create key to break for loop
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
                trackbar.save_values()
        elif key == ord("q"):
                cv2.destroyAllWindows()
                break

        rawCapture.truncate(0)
