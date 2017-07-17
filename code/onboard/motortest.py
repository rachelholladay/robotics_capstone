"""
File for testing motor actuation
"""

from motors import Motors

import time

m = Motors()

m.write(0)
time.sleep(5)
m.write(1)
