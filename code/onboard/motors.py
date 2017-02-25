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
from encoders import Encoders

'''
Constants
'''
# Encoder shaft rotations per motive shaft rotation
GEAR_RATIO = 210.59
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
		self.setSpeedExit = False
		self.setSpeedExited = True
		self.pwms = np.array([0,0,0,0])
		self.prevErrors = np.array([0.0,0.0])

	def turnOffMotors():
		'''
		Use this before turning off the Pi or motors will not stop
		'''
		self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
		self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

	def setSpeed(speedX, speedY):
		'''
		0 is x axis, 1 is y axis
		speed is in rpm
		'''
		# Kill other instances of pidSpeed
		self.setSpeedExit = True
		while self.setSpeedExited is False:
			pass
		self.setSpeedEixt = False
		self.setSpeedExited = False
		t = threading.Thread(target=self.pidSpeed, args=(speedX, speedY))
		t.start()

	def pidSpeed(speedX, speedY):
		targets = np.array([speedX, speedX, speedY, speedY])
		while self.setSpeedExit is False:
			currSpeeds = self.getSpeeds()
			# Compute new PWM values using PID control
			errors = targets-currSpeeds
			self.pwms = 
			self.pwmX += int(errors[0]*kPS + (errors[0]-prevErrors[0])*kDS)
			self.pwmY += int(errors[1]*kPS + (errors[1]-prevErrors[1])*kDS)
			prevErrors = errors
			# Execute new PWM values
			if pwmX > 0:
				self.motors[0].run(Adafruit_MotorHAT.FORWARD)
				self.motors[1].run(Adafruit_MotorHAT.FORWARD)
			else:
				self.motors[0].run(Adafruit_MotorHAT.BACKWARD)
				self.motors[1].run(Adafruit_MotorHAT.BACKWARD)
			if pwmX > 0:
				self.motors[0].run(Adafruit_MotorHAT.FORWARD)
				self.motors[1].run(Adafruit_MotorHAT.FORWARD)
			else:
				self.motors[0].run(Adafruit_MotorHAT.BACKWARD)
				self.motors[1].run(Adafruit_MotorHAT.BACKWARD)
			self.motors[0].setSpeed(pwmX)
			self.motors[1].setSpeed(pwmX)
		self.setSpeedExited = True

	def followVector(x, y):
		'''
		X component of vector
		'''

