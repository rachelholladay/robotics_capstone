"""
Contains constants required across subsystems
"""

# Small number for float comparison
SIGMA = 0.0001

################ COMMUNICATION SUBSYSTEM ################
# Communication values - used both onboard and offboard
OFFBOARD_IP = '127.0.0.1'
PORT = 5555
BUFFER_SIZE = 1024

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
BOTTOM_BORDER = 0 
TOP_BORDER = 10
LEFT_BORDER = 0
RIGHT_BORDER = 10
