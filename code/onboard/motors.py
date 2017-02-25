'''
Motor control for robot
Motors 1 and 2 are x direction motion
Motors 3 and 4 are y direction motion
'''

#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor 
import time
import threading
import numpy as np
from math import sqrt
from encoders import Encoders

'''
Constants
'''
# Encoder shaft rotations per motive shaft rotation
GEAR_RATIO = 210.59
# Counts per revolution
CPR = 12
# Encoder pin numbers
a1 = 4
a2 = 17
a3 = 18
a4 = 27
b1 = 22
b2 = 23
b3 = 24
b4 = 25
# Speed PID constants
kPS = 0.4
kDS = 0.1
# Period to get speed (in seconds)
speedDelay = 0.050
# Linear speed (in rpm)
constSpeed = 80


class Motors(object):
	def __init__(self, address):
		'''
		Instantiates the Motors class with the specified I2C address
		'''
		self.mh = Adafruit_MotorHAT(addr=address)
		self.motors = [mh.getMotor(1), mh.getMotor(1), 
			mh.getMotor(1), mh.getMotor(1)]
		self.encoders = [Encoders(a1, b1), Encoders(a2, b2), 
			Encoders(a3, b3), Encoders(a3, b3)]
		self.setSpeedsExit = False
		self.setSpeedsExited = True
		self.pwms = np.array([0,0,0,0])
		self.prevErrors = np.array([0.0,0.0])

	def turnOffMotors(self):
		'''
		Use this before turning off the Pi or motors will not stop
		'''
		self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

	def setSpeeds(self, speedX, speedY):
		'''
		0 is x axis, 1 is y axis
		speed is in rpm
		'''
		# Kill other instances of pidSpeed
		self.setSpeedsExit = True
		while self.setSpeedsExited is False:
			pass
		self.setSpeedsEixt = False
		self.setSpeedsExited = False
		t = threading.Thread(target=self.pidSpeed, args=(speedX, speedY))
		t.start()

	def pidSpeed(self, speedX, speedY):
		targets = np.array([speedX, speedX, speedY, speedY])
		while self.setSpeedsExit is False:
			currSpeeds = self.getSpeeds()
			# Compute new PWM values using PID control
			errors = targets-currSpeeds
			self.pwms += (errors*kPS + (errors-prevErrors)*kDS).astype(int)
			prevErrors = errors
			# Execute new PWM values
			# Direction
			if pwmX > 0:
				self.motors[0].run(Adafruit_MotorHAT.FORWARD)
				self.motors[1].run(Adafruit_MotorHAT.FORWARD)
			else:
				self.motors[0].run(Adafruit_MotorHAT.BACKWARD)
				self.motors[1].run(Adafruit_MotorHAT.BACKWARD)
			if pwmY > 0:
				self.motors[2].run(Adafruit_MotorHAT.FORWARD)
				self.motors[3].run(Adafruit_MotorHAT.FORWARD)
			else:
				self.motors[2].run(Adafruit_MotorHAT.BACKWARD)
				self.motors[3].run(Adafruit_MotorHAT.BACKWARD)
			# Speed
			for x in xrange(0,4):
				self.motors[x].setSpeed(pwms[x])
		self.setSpeedsExited = True

	def getSpeeds(self):
		ticks = np.array([encoders[0].ticksA, encoders[1].ticksA, 
			encoders[2].ticksA, encoders[3].ticksA])
		currTime = time.clock()
		time.sleep(speedDelay)
		ticks -= np.array([encoders[0].ticksA, encoders[1].ticksA, 
			encoders[2].ticksA, encoders[3].ticksA])
		timeDiff = currTime-time.clock()
		return ((ticks/timeDiff)/CPR)/GEAR_RATIO


	def followVector(self, x, y):
		'''
		x and y are 2 components of a vector that the robot will attempt to follow
		'''
		length = sqrt(x^2+y^2)
		self.setSpeeds(x/constSpeed, y/constSpeed)

