"""
Motors class. Contains code controlling and accessing motor data.
"""
from __future__ import print_function

import time
import threading
import atexit

import numpy as np
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import RPi.GPIO as GPIO

from math import sqrt
from encoders import Encoder
from utils import constants as cst


"""
Constants for motor operation
"""
# Motor pin numbers: 4x4 numpy matrix.
# Each row is a motor [ENC_A1, ENC_A2 ENC_A3, ENC_A4]
PINS = np.array([24, 23, 18, 20])
WRITINGPINS = np.array([20,19,21])
WRITINGPWR = 5
WRITINGTIME = 0.1 # default .25, .1 is improvement
WRITINGPULSETIME = 0.005
PULSEPERIOD = 2
UNWRITINGTIME = 0.08
# How long to wait for motors to stop and reissue commands
STOP_TIME = 0.01
# Speed PID constants
kP = 4
kD = 0.1

class Motors(object):
    """
    Implements capabilities for accessing and powering various motors for
    motion and writing implement actuation.

    Actuation of the writing implement is split into two stages:
    Implement enabled:
        The writing implement is commanded downwards such that the marker
        hits the writing surfaces. Experimentation showed that minute bumps
        in the surface caused the implement to be pushed upwards during long
        periods of operation. To resolve this, the implement is 'pulsed'
        downwards at fixed intervals. This prevents the motor from burning out
        attempting to apply too much pressure into the ground, while 
        consistent marker pressure on the writing surface.
    Implement disabled:
        The writing implement is lifted upwards.
    """
    def __init__(self):
        """
        Instantiates arrays and encoder objects
        """
        mh = mh = Adafruit_MotorHAT(addr=0x60)
        e = [None, None, None, None]

        for motor in xrange(0,4):
            # Init encoders
            ePin = PINS[motor]
            if ePin is not None:
                e[motor] = Encoder(ePin)
            else:
                e[motor] = Encoder(-1)

        # Set GPIO pins for writing implement
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(WRITINGPINS[0], GPIO.OUT)
        GPIO.setup(WRITINGPINS[1], GPIO.OUT)
        GPIO.setup(WRITINGPINS[2], GPIO.OUT)
        self.pwm = GPIO.PWM(WRITINGPINS[2], 490)
        self.pwm.start(0)

        self.encoders = e
        self.prevErrors = np.array([0.0,0.0,0.0,0.0])

        # Thread exit flags
        self.stopFlag = False
        self.currThread = None
        self.motors = [
                        mh.getMotor(1), 
                        mh.getMotor(2), 
                        mh.getMotor(3), 
                        mh.getMotor(4)
                      ]
        atexit.register(self.stopMotors)

        # Writing implement starts off unactuated
        self.isWriting = False
        self.writingThread = None

        self._debug = 1

    def __del__(self):
        """
        Ends operation by stopping motors and closing GPIO pins
        """
        self.stopMotors()
        GPIO.cleanup()

    def setSpeed(self, targets):
        """
        Sets the speed of each motor
        @param targets 4-element array containing motor target speeds
        """
        self.stopFlag = False
        self.currThread = threading.Thread(target=self.pidSpeed, 
                                           args=(targets, ))
        self.currThread.start()

    def pidSpeed(self, targets):
        """
        Set speeds for all motors.
        @param speeds numpy array of rpms for motors
        """
        pwms = np.array([0.0, 0.0, 0.0, 0.0])

        while self.stopFlag is False:
            currSpeeds = self.getSpeeds()
            if self._debug:
                print(currSpeeds)

            # Compute new PWM values using PID control
            errors = targets - currSpeeds
            pwms += (errors * kP + \
                    (errors - self.prevErrors) * kD).astype(int)
            self.prevErrors = errors

            # Execute new PWM values
            for motor in xrange(0,4):
                self.commandMotor(motor, pwms[motor])

        self.stopFlag = False

    def getSpeeds(self):
        """
        Reads encoder rpm for all motors
        @return array containing motor rpms
        """
        return np.array([self.encoders[0].rpms(), self.encoders[1].rpms(),
                         self.encoders[2].rpms(), self.encoders[3].rpms()])

    def commandMotor(self, motorNum, cntrl):
        """
        @param motorNum between 0-3
        @param cntrl between -100 and 100 percent duty cycle, negative means 
                     backwards rotation
        """
        pwm = max(min(255, cntrl), -255)
        currMotor = self.motors[motorNum]

        if pwm < 0:
            currMotor.run(Adafruit_MotorHAT.FORWARD)
        else:
            currMotor.run(Adafruit_MotorHAT.BACKWARD)

        currMotor.setSpeed(abs(pwm))

    def stopMotors(self):
        """
        Stops all motor movements
        """
        self.stopFlag = True

        if self.currThread is not None:
            self.currThread.join()

        for motorNum in xrange(0,4):
            self.motors[motorNum].run(Adafruit_MotorHAT.RELEASE)

    	time.sleep(STOP_TIME)

    def enableWrite(self):
        """
        Enables the writing implement
        """
        self.write(cst.WRITE_ENABLE)

    def disableWrite(self):
        """
        Disables the writing implement
        """
        self.write(cst.WRITE_DISABLE)

    def pulseWrite(self):
        """
        Pulses the writing implement at a fixed interval once it has been
        actuated. This helps maintain consistent marker pressure on the ground
        without burning out the writing implement motor.
        """
        while self.isWriting is True:
            time.sleep(PULSEPERIOD)

            GPIO.output(WRITINGPINS[0], GPIO.HIGH)
            GPIO.output(WRITINGPINS[1], GPIO.LOW)

            self.pwm.ChangeDutyCycle(WRITINGPWR)
            time.sleep(WRITINGPULSETIME)
            self.pwm.ChangeDutyCycle(0)

            GPIO.output(WRITINGPINS[0], GPIO.LOW)

    def write(self, state):
        """
        @param state 0 is start writing, 1 is stop writing
        """
        if (not state) and (not self.isWriting):
            if self._debug:
                print("writing")
            GPIO.output(WRITINGPINS[0], GPIO.HIGH)
            GPIO.output(WRITINGPINS[1], GPIO.LOW)

            self.pwm.ChangeDutyCycle(WRITINGPWR)
            time.sleep(WRITINGTIME)
            self.pwm.ChangeDutyCycle(0)
            GPIO.output(WRITINGPINS[0], GPIO.LOW)

            self.isWriting = True
            t = threading.Thread(target=self.pulseWrite)
            self.writingThread = t
            t.start()

            return

        if state and self.isWriting:
            if self._debug:
                print("stop writing")

            # Wait for writing thread to quit
            self.isWriting = False
            self.writingThread.join()
            
            GPIO.output(WRITINGPINS[0], GPIO.LOW)
            GPIO.output(WRITINGPINS[1], GPIO.HIGH)

            self.pwm.ChangeDutyCycle(WRITINGPWR)
            time.sleep(UNWRITINGTIME)
            self.pwm.ChangeDutyCycle(0)
            GPIO.output(WRITINGPINS[1], GPIO.LOW)

            return
