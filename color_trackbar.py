import cv2 as cv
import os
import json

from cv2 import WINDOW_AUTOSIZE

slider_max = 255
title_window = "Threshold Values"


class Color_Trackbar:
    def __init__(self, name) -> None:
        self.img = [0]
        self.name = name

        self.H_l, self.S_l, self.V_l = 0, 0, 0
        self.H_h, self.S_h, self.V_h = 255, 255, 255

        self.__color_low = (0, 0, 0)
        self.__color_high = (255, 255, 255)

    def GUI(self):
        # Create a window for the trackbars
        cv.namedWindow(title_window, WINDOW_AUTOSIZE)
        cv.resizeWindow(title_window, 600, 400)

        cv.createTrackbar('H_l', title_window, self.H_l,
                          slider_max, self.__H_l_change)
        cv.createTrackbar('S_l', title_window, self.S_l,
                          slider_max, self.__S_l_change)
        cv.createTrackbar('V_l', title_window, self.V_l,
                          slider_max, self.__V_l_change)
        cv.createTrackbar('H_h', title_window, self.H_h,
                          slider_max, self.__H_h_change)
        cv.createTrackbar('S_h', title_window, self.S_h,
                          slider_max, self.__S_h_change)
        cv.createTrackbar('V_h', title_window, self.V_h,
                          slider_max, self.__V_h_change)

    def in_range(self):
        self.__color_low = (self.H_l, self.S_l, self.V_l)
        self.__color_high = (self.H_h, self.S_h, self.V_h)

        # Apply the value obtained from trackbar and apply to the thresholding
        self.mask = cv.inRange(self.img, self.__color_low, self.__color_high)

        cv.imshow('Mask', self.mask)

    def __H_l_change(self, val: int):
        self.H_l = val
        self.in_range()

    def __S_l_change(self, val: int):
        self.S_l = val
        self.in_range()

    def __V_l_change(self, val: int):
        self.V_l = val
        self.in_range()

    def __H_h_change(self, val: int):
        self.H_h = val
        self.in_range()

    def __S_h_change(self, val: int):
        self.S_h = val
        self.in_range()

    def __V_h_change(self, val: int):
        self.V_h = val
        self.in_range()

    def load_values(self) -> bool:
        with open("config.json", "r") as f:
            config = json.load(f)

        if self.name in config:
            c = config[self.name]["color"]
            self.H_l, self.S_l, self.V_l = c["low"]["H"], c["low"]["S"], c["low"]["V"]
            self.H_h, self.S_h, self.V_h = c["high"]["H"], c["high"]["S"], c["high"]["V"]

            return True
        else:
            return False

    def save_values(self):
        with open("config.json", "r") as f:
            config = json.load(f)

        if self.name in config:
            config[self.name] = {
                "color":
                {
                    "low":{"H": self.__color_low[0], "S": self.__color_low[1], "V": self.__color_low[2]},
                    "high":{"H": self.__color_high[0], "S": self.__color_high[1], "V": self.__color_high[2]}
                },
                "threshold":config[self.name]["threshold"]
            }
        else:
            config[self.name] = {
                "color":
                {
                    "low":{"H": self.__color_low[0], "S": self.__color_low[1], "V": self.__color_low[2]},
                    "high":{"H": self.__color_high[0], "S": self.__color_high[1], "V": self.__color_high[2]}
                },
                "threshold":{}
            }
        with open("config.json", "w") as f:
            json.dump(config, f, indent=4)

    def update_img(self, image):
        self.img = image
        cv.imshow('Figure', self.img)
        self.mask = cv.inRange(self.img, self.__color_low, self.__color_high)
        cv.imshow('Mask', self.mask)



def color_trackbar():
    cap = cv.VideoCapture(0)
    ret, img = cap.read()
    t = Color_Trackbar(img)
    t = Color_Trackbar(img)
    t.GUI()
    while True:
        ret, img = cap.read()

        #t.load_values()
        t.update_img(img)
        k = cv.waitKey(1)

        if k == ord('s'):
            t.save_values(red)
        elif k == 27:
            cv.destroyAllWindows()
            break


if __name__ == '__main__':
    color_trackbar()
