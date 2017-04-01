'''
Encoders for the motors
'''

import RPi.GPIO as GPIO
import time

# Maximum amount of time (s) to wait for an encoder tick
MAX_WAIT = 0.5
# Minimum number of ticks to average RPM over
MIN_TICKS = 20
# Encoder shaft rotations per motive shaft rotation
GEAR_RATIO = 297.92
# Counts per revolution
CPR = 6*GEAR_RATIO
# Seconds per minute
SPM = 60
class Encoder(object):

    def __init__(self, outA, outB=-1):
        '''
        Instantiates an encoder object
        If outA is negative, we're not using this encoder
        '''
        GPIO.setmode(GPIO.BCM)
        if outA > 0:
            GPIO.setup(outA, GPIO.IN)
        if outB > 0:
            GPIO.setup(outB, GPIO.IN)

        self.outA = outA
        self.outB = outB

    def rpms(self):
        '''
        Function for getting RPM of motor
        '''
        # If encoder is not being used, return -1
        if self.outA < 0:
            return -1

        # Comput RPM
        prevState = 0
        ticks = 0
        startTime = time.clock()
        while ticks < MIN_TICKS:
            currState = GPIO.input(self.outA)
            if ((prevState is 0) and (currState is 1)) or \
                ((prevState is 1) and (currState is 0)):
                ticks += 1
            prevState = currState
            if (time.clock()-startTime) > MAX_WAIT:
                return 0.0
        timeDiff = time.clock() - startTime
        return (ticks/CPR)/(timeDiff/SPM)
