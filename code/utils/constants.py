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
BAD_IP = '192.168.0.24'
PORT = 10000
BUFFER_SIZE = 2048
MESSAGE_TIMEOUT = 4 # max delay in seconds between messages before onboard stops

################ LOCOMOTION SUBSYSTEM ################
# Distance the robot will stop from a target waypoint. Increasing this value 
# decreases overall accuracy, but setting it too low will cause the robot to
# "vibrate" in place by trying to constantly correct for minor localization
# errors.
STOP_DIST = 0.25

# Distance threshold for robot-robot collision detection. If the robots are
# closer than this threshold, robots will adhere to incoming collision detection
# rules.
COLLISION_BUFFER = 2.5

# Workaround for demo space ~y=5 having gap in wood underneath due to uneven
# floor. When within the GAP_BUFFER range the GAP_LOCATION, writing tool will
# automatically disable itself for the duration, regardless of the drawing
# state. This only occurs if GAP_ENABLED is true
GAP_ENABLED = True
GAP_LOCATION = 4.8 # along the y-axis of the demo space
GAP_BUFFER = 0.5


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
