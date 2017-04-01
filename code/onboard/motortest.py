from motors import Motors

import time

m = Motors()

for motor in xrange(0,4):
	m.commandMotor(motor, 100)

time.sleep(5)

m.stopMotors()