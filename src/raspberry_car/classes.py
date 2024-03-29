#!/usr/bin/env python
import json
from math import pi

import cv2
import RPi.GPIO as GPIO


class Car():
    """Creates a Car object.

    Parameters
    ----------
    left
        A tuple of the pins of the left wheel in the form of (enable, pin1,
    pin2)
    right
        A tuple of the pins of the right wheel in the form of (enable, pin1,
    pin2)
    encoder_left
        A tuple of of the pins of the left encoder in the form of (digital pin,
    diameter of the wheel, number of holes in the encoder wheel)
        Default is set to all -1 (off)
    encoder_left
        A tuple of of the pins of the right encoder in the form of
    (digital pin, diameter of the wheel, number of holes in the encoder wheel)
        Default is set to all -1 (off)
    diameter
        The distance between the wheels
    """
    def __init__(self, left, right,
                 encoder_left=(-1, -1, -1), encoder_right=(-1, -1, -1),
                 diameter=0):
        # Setting up the Left Wheel
        self.left_motor = Wheel(left[0], left[1], left[2])

        # Setting up the Right Wheel
        self.right_motor = Wheel(right[0], right[1], right[2])

        # Setting up Left Encoder
        if encoder_left[2] != -1:
            self.encoder_left = Encoder(encoder_left[0], encoder_left[1],
                                        encoder_left[2])
        else:
            self.encoder_left = -1

        # Setting up Right Encoder
        if encoder_right[2] != -1:
            self.encoder_right = Encoder(encoder_right[0], encoder_right[1],
                                         encoder_right[2])
        else:
            self.encoder_right = -1

        # Setting up diameter
        self.diameter = diameter

    def setup(self, pwm=False, freq=1000):
        """
        Sets up the GPIO modes of the pins

        Parameters
        ----------
        pwm
            Decides if pwm is needed or not, by default False
        freq
            Setting what frequency of the pwm signal, default is 1000 Hz
        """
        # Motors
        self.left_motor.setup(pwm, freq)
        self.right_motor.setup(pwm, freq)

        # Left Encoder if installed
        if self.encoder_left != -1:
            self.encoder_left.setup()

        # Right Encoder if installed
        if self.encoder_right != -1:
            self.encoder_right.setup()

    def reset_encoders(self):
        """
        Resets the counters of both encoders
        """
        assert self.encoder_right != -1 or self.encoder_left != -1,\
            "Encoders aren't Installed properly"
        self.encoder_right.reset()
        self.encoder_left.reset()

    def get_distance(self):
        """
        Returns the distance traveled by both wheels

        Returns
        -------
        Tuple
        Distance traveled by the left and right wheels
        (left, right)
        """
        assert self.encoder_right != -1 or self.encoder_left != -1,\
            "Encoders aren't Installed properly"
        right = self.encoder_right.get_distance()
        left = self.encoder_left.get_distance()

        return left, right

    def get_counter(self):
        """
        Returns the counter measured by both encoder

        Returns
        -------
        Tuple
        Counter measured by the left and right encoder
        (left, right)
        """
        assert self.encoder_right != -1 or self.encoder_left != -1,\
            "Encoders aren't Installed properly"
        right = self.encoder_right.get_counter()
        left = self.encoder_left.get_counter()

        return left, right

    def update_encoders(self):
        """
        Update both encoders
        """
        assert self.encoder_right != -1 or self.encoder_left != -1,\
            "Encoders aren't Installed properly"
        self.encoder_right.update()
        self.encoder_left.update()

    def angle_to_distance(self, angle, mode):
        """
        Returns the distance the wheel need to travel for the car to turn at an
        angle angle using the turning mode mode.

        Parameters
        ----------
        angle
            The angle to turn the car at in radians.
        mode
            The turning mode of the car
            Read documentation of turn_right or turn_left method for more
        information about the modes.

        Returns
        -------
        None
        """
        if angle < 0:
            raise ValueError("Invalid Angle")

        if mode == 0:
            return 2*angle*self.diameter/2
        if mode == 1:
            return 2*angle*self.diameter
        else:
            raise ValueError("Invalid Mode")

    def move_car(self, speed_left, speed_right):
        """
        Move the Car according to the speeds

        Prameters
        ---------
        speed_left
            The speed of the left wheel
            Raises a ValueError if it is greater than 100 or less than -100
        speed_right
            The speed of the right wheel
            Raises a ValueError if it is greater than 100 or less than -100
        """
        self.set_speed(speed_left, "left")
        self.set_speed(speed_right, "right")

    def set_speed(self, speed, side):
        """
        Set the speed of a specific wheel according to side. The wheel will go
        the other way when it is a negative value

        Parameters
        ----------
        speed
            The speed to set the wheel into
            The wheel will not turn faster when speed is < -100 or > 100
        side
            Can be either "left" or "right"
            Raises ValueError if neither was given
        """
        # Raising Error if invalid side
        if side != "left" and side != "right":
            raise ValueError("Invalid Side")

        # Deciding which wheel is beeing used
        if side == "left":
            wheel = self.left_motor
        else:
            wheel = self.right_motor

        wheel.set_speed(speed)

    def forward(self, speed=100, distance=0):
        """
        Moves the car forwards according to speed.

        If distance is specified, the car will move forward for that amount of
        distance The distance traveled is measured in the unit of wheel
        diameter given to the encoder. Both Encoders must be installed for
        distance travel to work.

        Prameters
        ---------
        speed
            The speed to move forward in, default is 100.
            Raises a ValueError if speed is greater than 100 or less than 0.
        distance
            The distance the car should travel.
            Raises a ValueError if distance is negative.
        """
        if speed < 0:
            raise ValueError("Invalid Speed")
        if distance < 0:
            raise ValueError("Invalid Distance")

        if distance == 0:
            self.move_car(speed, speed)
        else:
            self.reset_encoders()
            distance_traveled = self.get_distance()
            speedL, speedR = (speed, speed)
            while distance_traveled[0] <= distance or\
                    distance_traveled[1] <= distance:
                self.update_encoders()
                distance_traveled = self.get_distance()
                self.move_car(speedL, speedR)

                if distance_traveled[0] > distance_traveled[1]:
                    speedL = 0.5*speed
                    speedR = speed
                elif distance_traveled[1] > distance_traveled[0]:
                    speedL = speed
                    speedR = 0.5*speed
                else:
                    speedL = speed
                    speedR = speed

    def backward(self, speed=100, distance=0):
        """Moves the car backwards according to speed.

        If distance is specified, the car will move backward for that amount of
        distance. The distance traveled is measured in the unit of wheel
        diameter given to the encoder. Both Encoders must be installed for
        distance travel to work.

        Prameters
        ---------
        speed
            The speed to move backwards in, default is 100.
            Raises a ValueError if speed is greater than 100 or less than 0.
        distance
            The distance the car should travel.
            Raises a ValueError if distance is negative.
        """
        if speed < 0:
            raise ValueError("Invalid Speed")
        if distance < 0:
            raise ValueError("Invalid Distance")

        if distance == 0:
            self.move_car(-speed, -speed)
        else:
            self.reset_encoders()
            distance_traveled = self.get_distance()
            speedL, speedR = (-speed, -speed)
            while distance_traveled[0] <= distance or\
                    distance_traveled[1] <= distance:
                self.update_encoders()
                distance_traveled = self.get_distance()
                self.move_car(speedL, speedR)

                if distance_traveled[0] > distance_traveled[1]:
                    speedL = -0.5*speed
                    speedR = -speed
                elif distance_traveled[1] > distance_traveled[0]:
                    speedL = -speed
                    speedR = -0.5*speed
                else:
                    speedL = -speed
                    speedR = -speed

    def turn_left(self, speed=100, mode=0, angle=0):
        """Turns the car left according to speed.

        If angle is specified, the car will turn at that angle in radians
        The diameter of the car must be specified for the turning angle to work

        Prameters
        ---------
        speed
            The speed to turn the in, default is 100
            Raises a ValueError if speed is greater than 100 or less than 0
        mode
            Decides how the car will turn
            if mode is 0, one wheel will move forward and the other will move
        backwards (turn in the same spot)
            if mode is 1, one wheel will move forward and the other will stop
        (turn while still moving forward)
        angle
            The angle to turn the car
            Raises a ValueError if speed is less than 0
        """
        # Checking Conditions
        if speed < 0:
            raise ValueError("Invalid Speed")
        if angle < 0:
            raise ValueError("Invalid Angle")

        # Checking If angle is specified
        if angle == 0:
            # Checking Mode
            if mode == 0:
                self.move_car(-speed, speed)
            elif mode == 1:
                self.move_car(0, speed)
            else:
                raise ValueError("Invalid Mode")
        else:
            # Checking Mode
            if mode == 0:
                # Getting Initial Condition
                self.reset_encoders()
                distance = self.angle_to_distance(angle, 0)
                distance_traveled = self.get_distance()
                speedL, speedR = (-speed, speed)
                # Looping until completed
                while distance_traveled[0] <= distance or\
                        distance_traveled[1] <= distance:
                    # Update Condition
                    self.update_encoders()
                    distance_traveled = self.get_distance()
                    self.move_car(speedL, speedR)

                    # Speed Logic
                    if distance_traveled[0] > distance_traveled[1]:
                        speedL = -0.5*speed
                        speedR = speed
                    elif distance_traveled[1] > distance_traveled[0]:
                        speedL = -speed
                        speedR = 0.5*speed
                    else:
                        speedL = -speed
                        speedR = speed
            elif mode == 1:
                # Getting Initial Condition
                self.reset_encoders()
                distance = self.angle_to_distance(angle, 1)
                distance_traveled = self.get_distance()
                speedL, speedR = (0, speed)
                # Looping until completed
                while distance_traveled[1] <= distance:
                    # Update Condition
                    self.update_encoders()
                    distance_traveled = self.get_distance()
                    self.move_car(speedL, speedR)
            else:
                raise ValueError("Invalid Mode")

    def turn_right(self, speed=100, mode=0, angle=0):
        """Turns the car right according to speed.

        If angle is specified, the car will turn at that angle in radians
        The diameter of the car must be specified for the turning angle to work

        Prameters
        ---------
        speed
            The speed to turn the in, default is 100
            Raises a ValueError if speed is greater than 100 or less than 0
        mode
            Decides how the car will turn
            if mode is 0, one wheel will move forward and the other will move
        backwards (turn in the same spot)
            if mode is 1, one wheel will move forward and the other will stop
        (turn while still moving forward)
        angle
            The angle to turn the car
            Raises a ValueError if speed is less than 0
        """
        # Checking Conditions
        if speed < 0:
            raise ValueError("Invalid Speed")
        if angle < 0:
            raise ValueError("Invalid Angle")

        # Checking If angle is specified
        if angle == 0:
            # Checking Mode
            if mode == 0:
                self.move_car(speed, -speed)
            elif mode == 1:
                self.move_car(speed, 0)
            else:
                raise ValueError("Invalid Mode")
        else:
            # Checking Mode
            if mode == 0:
                # Getting Initial Condition
                self.reset_encoders()
                distance = self.angle_to_distance(angle, 0)
                distance_traveled = self.get_distance()
                speedL, speedR = (speed, -speed)
                # Looping until completed
                while distance_traveled[0] <= distance or\
                        distance_traveled[1] <= distance:
                    # Update Condition
                    self.update_encoders()
                    distance_traveled = self.get_distance()
                    self.move_car(speedL, speedR)

                    # Speed Logic
                    if distance_traveled[0] > distance_traveled[1]:
                        speedL = 0.5*speed
                        speedR = -speed
                    elif distance_traveled[1] > distance_traveled[0]:
                        speedL = speed
                        speedR = -0.5*speed
                    else:
                        speedL = speed
                        speedR = -speed
            elif mode == 1:
                # Getting Initial Condition
                self.reset_encoders()
                distance = self.angle_to_distance(angle, 1)
                distance_traveled = self.get_distance()
                speedL, speedR = (speed, 0)
                # Looping until completed
                while distance_traveled[0] <= distance:
                    # Update Condition
                    self.update_encoders()
                    distance_traveled = self.get_distance()
                    self.move_car(speedL, speedR)
            else:
                raise ValueError("Invalid Mode")

    def stop(self):
        """
        Stops the car
        """
        self.move_car(0, 0)


class Encoder:
    """
    Creates an Encoder object

    Parameters
    ----------
    pin
        The pin number of the digital pin
    d
        The diameter of the wheel
    holes
        The number of holes the encoder wheel has
    """
    def __init__(self, pin, d, holes):
        self.count = 0
        self.diameter = d
        self.nHoles = holes
        self.digitalPin = pin

    def setup(self):
        """
        Setting up the GPIO pin modes
        """
        GPIO.setup(self.digitalPin, GPIO.IN)
        self.cStatus = GPIO.input(self.digitalPin)

    def __blink(self):
        """
        Executed when the encoder changes state
        """
        self.count += 1

    def reset(self):
        """
        Resets the counter in which the encoder state changes
        """
        self.count = 0

    def count_to_distance(self, cCounter):
        """Converts the amount of changes of encoder states into distance
        according to the unit of the diameter given.

        Parameters
        ----------
        cCounter
            The amount of state changes

        Returns
        -------
        int
            The distance calculated
        """
        ratio = 2*pi*self.diameter / (2*self.nHoles)
        return ratio * cCounter

    def distance_to_count(self, distance):
        """Converts distance according to the unit of the diameter given into
        the amount of changes of encoder states.

        Parameters
        ----------
        distance
            Distance in the unit of the diameter given

        Returns
        -------
        int
            The amount of state changes calculated
        """
        ratio = self.nHoles / (2*pi*self.diameter)
        return ratio * distance

    def update(self):
        """
        Updates the amount of state changes

        Returns
        -------
        None
        """
        status = GPIO.input(self.digitalPin)
        if self.cStatus != status:
            self.cStatus = status
            self.__blink()

    def get_counter(self):
        """
        Get the amount of state changes has occured

        Returns
        -------
        int
            The amount of state changes
        """
        return self.count

    def get_distance(self):
        """Get the distance traveled as recorded by the encoder.

        Returns
        -------
        int
            The distance traveled as recorded by the encoder in the unit of the
        diameter given.
        """
        return self.count_to_distance(self.count)


class Wheel:
    """Creates a wheel object.

    Parameters
    ----------
    enable
        Enable pin of the wheel.
    pin1
        One of GPIO pin number.
    pin2
        One of GPIO pin number.
    """
    def __init__(self, enable, pin1, pin2):
        self.enable = enable
        self.pin1 = pin1
        self.pin2 = pin2
        self.pwm = -1

    def set_speed(self, speed):
        """Set the speed of the wheel.

        Parameters
        ----------
        speed
            The speed to set the wheel into.
            Raises a ValueError if speed is greater than 100 or less than -100.
        """
        absSpeed = abs(speed)

        # Retrieving values from the wheel
        pin1 = self.pin1
        pin2 = self.pin2
        enable = self.enable
        pwm = self.pwm

        # Changes speed if pwm is set up
        if pwm != -1:
            if absSpeed > 100 or absSpeed < 0:
                absSpeed = 100
                pwm.start(absSpeed)
            if absSpeed == 0:
                pwm.stop()
            else:
                pwm.start(absSpeed)
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

    def setup_pwm(self, freq=1000):
        """Setting up the pwm object of the wheel if changing the speed of the
        wheels is needed with frequency freq (default 1000Hz).

        Parameters
        ----------
        freq
            Setting what frequency of the pwm signal, default is 1000 Hz
        """
        self.pwm = GPIO.PWM(self.enable, freq)

    def setup(self, pwm=False, freq=1000):
        """Sets up the GPIO modes of the pins.

        Parameters
        ----------
        pwm
            Decides if pwm is needed or not, by default False.
        freq
            Setting what frequency of the pwm signal, default is 1000 Hz.
        """
        GPIO.setup(self.enable, GPIO.OUT)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)

        if pwm:
            self.setup_pwm(freq)


class PID:
    """Creates a PID object.

    Parameters
    ----------
    baseline
        The baseline value in which to compared to
    K_P
        The constant value of P
    K_I
        The constant value of I
    K_D
        The constant value of D
    """
    def __init__(self, baseline, K_P, K_I, K_D):
        self.baseline = baseline
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.error = 0
        self.P = 0
        self.i_val = 0
        self.D = 0
        self.prev_error = 0

    def update(self, value):
        """
        Update P, I, D and error according to value

        Parameters
        ----------
        value
            The current value to be compared with

        Returns
        -------
        None
        """
        self.error = self.baseline - value
        self.P = self.error
        self.i_val += self.error
        self.D = self.error - self.prev_error
        self.prev_error = self.error

    def get_PID(self):
        """
        Get the PID value according to current P, I and D value

        Returns
        -------
        int
            The PID value
        """
        PID = self.K_P * self.P + self.K_I * self.i_val + self.K_D * self.D
        return PID


class Colour:
    """
    Creates a colour object that filters out a specific line colour

    Parameters
    ----------
    colour
        What colour to filter
    config
        The config where the threshold HSV values is stored
    """
    def __init__(self, colour, config="config.json"):
        with open(config, "r") as f:
            c = json.load(f)

        if colour in c:
            self.colour = colour
            c = c[colour]["color"]

            self.H_l = c["low"]["H"]
            self.S_l = c["low"]["S"]
            self.V_l = c["low"]["V"]
            self.H_h = c["high"]["H"]
            self.S_h = c["high"]["S"]
            self.V_h = c["high"]["V"]

            self.__color_low = (self.H_l, self.S_l, self.V_l)
            self.__color_high = (self.H_h, self.S_h, self.V_h)
        else:
            raise ValueError("Invalid Colour")

    def get_image(self):
        """
        Returns the image array of the processed image

        Returns
        -------
        numpy.ndarray
            The processed image
        """
        return self.image

    def update_image(self, input_image):
        """
        Update the image precessed image used to input_image

        Parameters
        ----------
        input_image
            The image to change to

        Returns
        -------
        None
        """
        self.image = cv2.inRange(input_image, self.__color_low,
                                 self.__color_high)
        self.contours, self.hierarchy = cv2.findContours(
            self.image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

    def get_contours(self):
        """
        Returns the contours found

        Returns
        -------
        Tuple
            The contours found
        """
        return self.contours
