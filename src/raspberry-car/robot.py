#!/usr/bin/env python3
import RPi.GPIO as GPIO
from time import sleep, time
from classes import *

right_wheel = (12, 21, 20)
left_wheel = (18, 24, 23)

car = Car(left_wheel, right_wheel, (26, 6.5, 20), (19, 6.5, 20), 13.5)
encoderLeft = Encoder(26, 6.5, 20)
encoderRight = Encoder(19, 6.5, 20)

def setup():
    print("Program has Started")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    car.setup(True)
    encoderLeft.setup()
    encoderRight.setup()

def loop():
    while True:
        car.turn_right(100)
        encoderLeft.update()
        encoderRight.update()
        if encoderLeft.get_counter() >= 20 or encoderRight.get_counter() >= 20:
            print(encoderLeft.get_counter())
            print(encoderRight.get_counter())
            car.stop()
            break

def exit():
    car.move_car(0, 0)
    GPIO.cleanup()
    print("Program has Stopped")


if __name__ == '__main__':
    setup()
    try:
        loop()
    finally:
        exit()
