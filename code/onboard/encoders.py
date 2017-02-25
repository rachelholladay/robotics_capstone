'''
Encoders for the motors
'''

import RPi.GPIO as GPIO

class Encoder(object):
	'''
	Instantiates an encoder object
	'''
	def __init__(self, outA, outB):
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(outA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.setup(outB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		
		self.outA = outA;
		self.outB = outB;
		self.ticksA = 0;
		self.ticksB = 0;

		# Add GPIO interrupt events
		GPIO.add_event_detect(outA, GPIO.RISING, callback=self.tickA)
		GPIO.add_event_detect(outB, GPIO.RISING, callback=self.tickB)

	'''
	Callback functions for incrementing tick counters
	'''
	def tickA(channel):
		self.ticksA += 1
	def tickB(channel):
		self.ticksB += 1
