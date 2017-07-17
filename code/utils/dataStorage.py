"""
File containing data structs for passing information between subsystems
"""

from utils.geometry import DirectedPoint
from utils import constants as cst


class LocomotionData(object):
    """
    Contains fields for locomotion data for a single robot
    Locomotion data includes current and target position and orientations
    """
    def __init__(self, tf_robot=None, tf_target=None, stop_status=0,
        write_status=cst.WRITE_DISABLE):
        """
        Creates LocomotionData object with target, current position, and stop
        parameter.
        @param tf_robot DirectedPoint for robot current pos/orientation
        @param tf_target DirectedPoint for target current pos/orientation
        @param stop_status Boolean whether the robot should stop moving or not
        """
        self.tf_robot = tf_robot
        self.tf_target = tf_target
        self.write_status = write_status
        self.stop_status = stop_status


class LocalizationData:
    """
    Struct containing localization data for each of the 4 corners and 2 robots
    """
    def __init__(self):
        """
        Dictionary mapping AprilTag IDs to DirectedPoint objects.
        Keys best accessed via constants.TAG_[BOTTOM/TOP]_[LEFT/RIGHT]
          ex. constants.TAG_BOTTOM_LEFT

        Separate dictionaries exist for corner tags (constant during
        system operation) and robot tags (volatile during operation)
            self.corners = dict()
            self.robots = dict()
        """
        self.corners = {cst.TAG_BOTTOM_LEFT: DirectedPoint(valid=False),
                        cst.TAG_BOTTOM_RIGHT: DirectedPoint(valid=False),
                        cst.TAG_TOP_LEFT: DirectedPoint(valid=False),
                        cst.TAG_TOP_RIGHT: DirectedPoint(valid=False) }
        self.robots = {cst.TAG_ROBOT1: DirectedPoint(valid=False),
                       cst.TAG_ROBOT2: DirectedPoint(valid=False) }

class LineInputData(object):
    """
    Struct containing the input data, the lines and bounds
    """
    def __init__(self):
        self.vertical_bounds = [-1, -1]
        self.horizontal_bounds = [-1, -1]
        self.lines = []

class ErrorData(object):
    """
    Contains fields for any error messages to be sent to the robots
    """
    def __init__(self):
        pass
