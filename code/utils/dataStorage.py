"""
File containing data structs for passing information between subsystems
"""

from utils.geometry import DirectedPoint
from utils import constants as cst

class LocomotionData(object):
    """
    Contains fields for locomotion data for a single robot
    Locomotion data includes wheel motor commands as well as motor commands
    for the writing implement
    """
    def __init__(self):
      self.motor_command = [0, 0, 0, 0] # TODO rename 'command' more accurately

class LocalizationData:
    """
    Struct containing localization data for each of the 4 corners and 2 robots
    """
    def __init__(self):
        # Dictionary mapping AprilTag IDs to DirectedPoint objects.
        # Keys best accessed via constants.TAG_[BOTTOM/TOP]_[LEFT/RIGHT]
        #   ex. constants.TAG_BOTTOM_LEFT
        # Separate dictionaries exist for corner tags (constant during
        # system operation) and robot tags (volatile during operation)
        self.corners = dict()
        self.robots = dict()
        # self.corners = {cst.TAG_BOTTOM_LEFT: DirectedPoint(valid=False),
        #                 cst.TAG_BOTTOM_RIGHT: DirectedPoint(valid=False),
        #                 cst.TAG_TOP_LEFT: DirectedPoint(valid=False),
        #                 cst.TAG_TOP_RIGHT: DirectedPoint(valid=False) }
        # self.robots = {cst.TAG_ROBOT1: DirectedPoint(valid=False),
        #                cst.TAG_ROBOT2: DirectedPoint(valid=False) }


class ErrorData(object):
    """
    Contains fields for any error messages to be sent to the robots
    """
    def __init__(self):
        pass