'''
Locomotion subsystem
'''
from utils.dataStorage import LocomotionData

class LocomotionSystem(object):
    '''
    Contains locomotion subsystem.
    '''
    def __init__(self):
        self.locomotionData = LocomotionData()

    def updateLocomotion(self, localizationData, pathData):
        """
        Updates the LocomotionData command using the passed in localization
        data and path planning data
        @param localizationData LocalizationData struct
        @param pathData struct containing path planning data
        @return LocomotionData struct
        """
        return self.locomotionData

    def computeMotorCommands(self, start, goal):
        """
        Uses the starting and goal DirectedPoints to compute unnormalized
        and uncalibrated motor commands for each motor.
        Equations from: https://www.roboteq.com/index.php/component/easyblog/entry/driving-mecanum-wheels-omnidirectional-robots?Itemid=1208
        @param start DirectedPoint for the robot's current position/orientation
        @param goal DirectedPoint for the robot's goal position/orientation
        @return LocalizationData struct containing unnormalized and
            uncalibrated motor commands
        """
        command = LocomotionData()

        # Compute desired angle for the robot to translate at
        angle = 0

        # Compute robot speed [-1, 1]
        magnitude = 0

        # Compute rotation speed [0, 2pi]
        rotation_speed = 0


        return command



