"""
File containing data structs for passing information between subsystems
"""

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
        corners = [DirectedPoint(), DirectedPoint(), DirectedPoint(), \
            DirectedPoint()]
        robots = [DirectedPoint(), DirectedPoint()]


class ErrorData(object):
    """
    Contains fields for any error messages to be sent to the robots
    """
    def __init__(self):
        pass