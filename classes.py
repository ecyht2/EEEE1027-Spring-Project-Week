#!/usr/bin/env python
import RPi.GPIO as GPIO
from math import pi

class Car():
    """
    Creates a Car object

    Parameters
    ----------
    left
        A tuple of the pins of the left wheel in the form of (enable, pin1, pin2)
    right
        A tuple of the pins of the right wheel in the form of (enable, pin1, pin2)
    encoder_left
        A tuple of of the pins of the left encoder in the form of (digital pin, diameter of the wheel, number of holes in the encoder wheel)
        Default is set to all -1 (off)
    encoder_left
        A tuple of of the pins of the right encoder in the form of (digital pin, diameter of the wheel, number of holes in the encoder wheel)
        Default is set to all -1 (off)
    """
    def __init__(self, left, right, encoder_left = (-1, -1, -1), encoder_right = (-1, -1, -1)):
        # Setting up the Left Wheel
        self.left_motor = Wheel(left[0], left[1], left[2])

        # Setting up the Right Wheel
        self.right_motor = Wheel(right[0], right[1], right[2])

        # Setting up Left Encoder
        if encoder_left[2] != -1:
            self.encoder_left = Encoder(encoder_left[0], encoder_left[1], encoder_left[2])
        else:
            self.encoder_left = -1

        # Setting up Right Encoder
        if encoder_right[2] != -1:
            self.encoder_right = Encoder(encoder_right[0], encoder_right[1], encoder_right[2])
        else:
            self.encoder_right = -1

    def setup(self, pwm = False, freq = 1000):
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
        if self.encoder_left != -1
            self.encoder_left.setup()

        # Right Encoder if installed
        if self.encoder_right != -1
            self.encoder_right.setup()

    def reset_encoders(self):
        self.encoder_right

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
        Set the speed of a specific wheel according to side. The wheel will go the other way when it is a negative value

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

    def forward(self, speed = 100, distance = 0):
        """
        Moves the car forwards according to speed

        Prameters
        ---------
        speed
            The speed to move forward in, default is 100
            Raises a ValueError if speed is greater than 100 or less than 0
        """
        if speed < 0:
            raise ValueError("Invalid Speed")

        if distance = 0:
            self.move_car(speed, speed)
        else:
            self.reset_encoders()
            distance_traveled = self.get_distance()
            while distance_traveled[0] <= distance or distance_traveled[1] <= distance:
                self.update_encoders()
                distance_traveled = self.get_distance()
                self.move_car(speed, speed)

                

    def backward(self, speed = 100):
        """
        Moves the car backwards according to speed

        Prameters
        ---------
        speed
            The speed to move backwards in, default is 100
            Raises a ValueError if speed is greater than 100 or less than 0
        """
        if speed < 0:
            raise ValueError("Invalid Speed")
        self.move_car(-speed, -speed)

    def turn_left(self, speed = 100, mode = 0):
        """
        Turns the car left according to speed

        Prameters
        ---------
        speed
            The speed to turn the in, default is 100
            Raises a ValueError if speed is greater than 100 or less than 0
        mode
            Decides how the car will turn
            if mode is 0, one wheel will move forward and the other will move backwards (turn in the same spot)
            if mode is 1, one wheel will move forward and the other will stop (turn while still moving forward)
        """
        if speed < 0:
            raise ValueError("Invalid Speed")

        if mode == 0:
            self.move_car(-speed, speed)
        elif mode == 1:
            self.move_car(0, speed)
        else:
            raise ValueError("Invalid Mode")

    def turn_right(self, speed = 100, mode = 0):
        """
        Turns the car right according to speed

        Prameters
        ---------
        speed
            The speed to turn the in, default is 100
            Raises a ValueError if speed is greater than 100 or less than 0
        mode
            Decides how the car will turn
            if mode is 0, one wheel will move forward and the other will move backwards (turn in the same spot)
            if mode is 1, one wheel will move forward and the other will stop (turn while still moving forward)
        """
        if speed < 0:
            raise ValueError("Invalid Speed")

        if mode == 0:
            self.move_car(speed, -speed)
        elif mode == 1:
            self.move_car(speed, 0)
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
        self.count+=1

    def reset(self):
        """
        Resets the counter in which the encoder state changes
        """
        self.count = 0

    def count_to_distance(self, cCounter):
        """
        Converts the amount of changes of encoder states into distance according to the unit of the diameter given

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
        """
        Converts distance according to the unit of the diameter given into the amount of changes of encoder states

        Parameters
        ----------
        distance
            Distance in the unit of the diameter given

        Returns
        -------
        int
            The amount of state changes calculated
        """
        ratio = nHoles / (2*pi*self.diameter)
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
        """
        Get the distance traveled as recorded by the encoder

        Returns
        -------
        int
            The distance traveled as recorded by the encoder in the unit of the diameter given
        """
        return self.count_to_distance(self.count)

class Wheel:
    """
    Creates a wheel object

    Parameters
    ----------
    enable
        Enable pin of the wheel
    pin1
        One of GPIO pin number
    pin2
        One of GPIO pin number
    """
    def __init__(self, enable, pin1, pin2):
        self.enable = enable
        self.pin1 = pin1
        self.pin2 = pin2
        self.pwm = -1

    def set_speed(self, speed):
        """
        Set the speed of the wheel

        Parameters
        ----------
        speed
            The speed to set the wheel into
            Raises a ValueError if speed is greater than 100 or less than -100
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

    def setup_pwm(self, freq = 1000):
        """
        Setting up the pwm object of the wheel if changing the speed of the wheels is needed with frequency freq (default 1000Hz)

        Parameters
        ----------
        freq
            Setting what frequency of the pwm signal, default is 1000 Hz
        """
        self.pwm = GPIO.PWM(self.enable, freq)

    def setup(self, pwm = False, freq = 1000):
        """
        Sets up the GPIO modes of the pins

        Parameters
        ----------
        pwm
            Decides if pwm is needed or not, by default False
        freq
            Setting what frequency of the pwm signal, default is 1000 Hz
        """
        GPIO.setup(self.enable, GPIO.OUT)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)

        if pwm:
            self.setup_pwm(freq)

class PID:
    """
    Creates a PID object

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
        self.I = 0
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
        self.I += self.error
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
        PID = self.K_P*self.P + self.K_I*self.I + self.K_D*self.D
        return PID
