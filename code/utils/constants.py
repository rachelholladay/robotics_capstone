"""
Contains constants required across subsystems
"""

# Small number for float comparison
EPSILON = 0.0001
EPSILON_LARGE = 0.001

################ SYSTEM-WIDE CONSTANTS ################
BLUE_ID = 0
BAD_ID = 1

################ COMMUNICATION SUBSYSTEM ################
# Communication values - used both onboard and offboard
OFFBOARD_IP = '192.168.0.26' # NJ macbook ubuntu vm IP
BLUE_IP = '192.168.0.23'
BAD_IP = 'UPDATE_THIS'
PORT = 10000
BUFFER_SIZE = 2048
MESSAGE_TIMEOUT = 4 # max delay in seconds between messages before onboard stops

################ LOCOMOTION SUBSYSTEM ################
# Distance the robot will stop from a target waypoint. Increasing this value 
# decreases overall accuracy, but setting it too low will cause the robot to
# "vibrate" in place by trying to constantly correct for minor localization
# errors.
STOP_DIST = 0.25
# locomotion stop_status parameter values
ROBOT_MOVE = 0
ROBOT_STOP = 1

################ LOCALIZATION SUBSYSTEM ################
# Corners and robots are assigned to an AprilTag ID - this is to
# specify which corners correspond to specific Apriltags.
TAG_BOTTOM_LEFT = 4
TAG_BOTTOM_RIGHT = 3
TAG_TOP_LEFT = 1
TAG_TOP_RIGHT = 2
TAG_CORNERS = [TAG_BOTTOM_LEFT, TAG_BOTTOM_RIGHT,
                 TAG_TOP_LEFT, TAG_TOP_RIGHT]
TAG_ROBOT1 = 5
TAG_ROBOT2 = 6
TAG_ROBOTS = [TAG_ROBOT1, TAG_ROBOT2]

################ PLANNING SUBSYSTEM ################
# (top, right) border must be greater than (bottom, left) borders, respectively
BOTTOM_BORDER = 0 
TOP_BORDER = 10
LEFT_BORDER = 0
RIGHT_BORDER = 10
HORIZ_PAD = 0.2
VERT_PAD = 0.2
assert(TOP_BORDER > BOTTOM_BORDER)
assert(RIGHT_BORDER > LEFT_BORDER)

################ WRITING SUBSYSTEM ################
# Writing status for communication messages
WRITE_ENABLE = 0
WRITE_DISABLE = 1
