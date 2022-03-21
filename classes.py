#!/usr/bin/env python
import RPi.GPIO as GPIO
from math import pi

class Car():
    """
    """
    def __init__(self, left, right, encoder_left = (-1, -1, -1), encoder_right = (-1, -1, -1)):
        # Setting up the Left Wheel
        self.left_motor = {"enable": left[0], "pin1": left[1], "pin2": left[2]}
        self.left_motor["pwm"] = -1

        # Setting up the Right Wheel
        self.right_motor = {"enable": right[0], "pin1": right[1], "pin2": right[2]}
        self.right_motor["pwm"] = -1

        # Setting up Left Encoder
        if encoder_left[2] != -1:
            self.encoder_left = Encoder(encoder_left[0], encoder_left[1], encoder_left[2])

        # Setting up Right Encoder
        if encoder_right[2] != -1:
            self.encoder_right = Encoder(encoder_right[0], encoder_right[1], encoder_right[2])

    def setup(self, pwm = False):
        for i in self.left_motor.values():
            if i == -1:
                continue
            GPIO.setup(i, GPIO.OUT)
        for i in self.right_motor.values():
            if i == -1:
                continue
            GPIO.setup(i, GPIO.OUT)

        if pwm:
            self.setup_pwm("right")
            self.setup_pwm("left")

        try:
            self.encoder_left.setup()
            self.encoder_right.setup()
        except (NameError, AttributeError):
            pass

    def setup_pwm(self, side, freq = 1000):
        """
        Setting up the pwm object of a given wheel if changing the speed of the wheels is needed with frequency freq (default 1000Hz)

        side can be either "left" or "right"
        Raises ValueError if neither was given
        """
        if side == "left":
            self.left_motor["pwm"] = GPIO.PWM(self.left_motor["enable"], freq)
        elif side == "right":
            self.right_motor["pwm"] = GPIO.PWM(self.right_motor["enable"], freq)
        else:
            raise ValueError("Invalid Side")

    def move_car(self, speed_left, speed_right):
        """
        Move the Car according to the speeds

        Raises a ValueError if either speed is greater than 100 or less than -100
        """
        self.set_speed(speed_left, "left")
        self.set_speed(speed_right, "right")

    def set_speed(self, speed, side):
        """
        Set the speed of a specific wheel according to side

        Raises a ValueError if speed is greater than 100 or less than -100

        side can be either "left" or "right"
        Raises ValueError if neither was given
        """
        # Raising Error if invalid side
        if side != "left" and side != "right":
            raise ValueError("Invalid Side")

        absSpeed = abs(speed)

        # Deciding which wheel is beeing used
        if side == "left":
            wheel = self.left_motor
        else:
            wheel = self.right_motor

        # Retrieving values from the wheel
        pin1 = wheel.get("pin1")
        pin2 = wheel.get("pin2")
        enable = wheel.get("enable")

        # Changes speed if pwm is set up
        if wheel["pwm"] != -1:
            if absSpeed > 100 or absSpeed < 0:
                raise ValueError("Invalid Speed")
            if absSpeed == 0:
                wheel.get("pwm").stop()
            else:
                wheel.get("pwm").start(absSpeed)
        else:
            GPIO.output(enable, True)

        # Speed Direction logic
        if speed == 0:
            GPIO.output(pin1, False)
            GPIO.output(pin2, False)
        elif speed > 0:
            GPIO.output(pin1, True)
            GPIO.output(pin2, False)
        else:
            GPIO.output(pin1, False)
            GPIO.output(pin2, True)

    def forward(self, speed = 100):
        """
        Moves the car forwards according to speed (default is 100)

        Raises a ValueError if speed is greater than 100 or less than -100
        """
        if speed < 0:
            raise ValueError("Invalid Speed")

        self.move_car(speed, speed)

    def backward(self, speed = 100):
        """
        Moves the car backwards according to speed (default is 100)

        Raises a ValueError if speed is greater than 100 or less than -100
        """
        if speed < 0:
            raise ValueError("Invalid Speed")
        self.move_car(-speed, -speed)

    def turn_left(self, speed = 100):
        """
        Turns the car left according to speed (default is 100)

        Raises a ValueError if speed is greater than 100 or less than -100
        """
        if speed < 0:
            raise ValueError("Invalid Speed")
        self.move_car(speed, -speed)

    def turn_right(self, speed = 100):
        """
        Turns the car right according to speed (default is 100)

        Raises a ValueError if speed is greater than 100 or less than -100
        """
        if speed < 0:
            raise ValueError("Invalid Speed")
        self.move_car(-speed, speed)

    def stop(self):
        """
        Stops the car
        """
        self.move_car(0, 0)

class Encoder:
    def __init__(self, pin, d, holes):
        self.count = 0
        self.diameter = d
        self.nHoles = holes
        self.digitalPin = pin

    def setup(self):
        GPIO.setup(self.digitalPin, GPIO.IN)
        self.cStatus = GPIO.input(self.digitalPin)

    def blink(self):
        self.count+=1

    def reset(self):
        self.count = 0

    def count_to_distance(self, cCounter):
        ratio = 2*pi*self.diameter / (2*self.nHoles)
        return ratio * cCounter

    def distance_to_count(self, distance):
        ratio = nHoles / (2*pi*self.diameter)
        return ratio * distance

    def update(self):
        status = GPIO.input(self.digitalPin)
        if self.cStatus != status:
            self.cStatus = status
            self.blink()

    def get_counter(self):
        return self.count

    def get_distance(self):
        return self.count_to_distance(self.count)
