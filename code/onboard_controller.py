'''
Main onboard controller.
'''
import sys
import math

from messages import robot_commands_pb2
from onboard.robot_communication import RobotCommunication


class OnboardController(object):
    def __init__(self, robot_ip):
        '''
        Instantiates main subsystems based on input parameters
        '''
        self.robot_ip = robot_ip




    def test(self):
        pass

    def getMotorCommands(self, current, target):
        """
        Uses mechanum control equations to compute motor powers for each motor
            to move from a provided current position to a desired target 
            position.
        This function does not take into account whether or not the robot has
            reached the target, and does not scale speed based on distance to 
            target.
        Function provides motor powers to move along the vector between the
        current and target position/orientations.

        Motor powers are ordered 1-4, given a robot with motors in the
        following position, and forward defined as:
        #    1 ---- 2     .
        #    |      |    / \
        #    |      |     |
        #    3 ---- 4     |
        
        # Mecanum Control:
        # https://www.roboteq.com/index.php/component/easyblog/entry/driving-mecanum-wheels-omnidirectional-robots?Itemid=1208
        # This pdf has accurate equations: http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf

        @param current DirectedPoint of robot current position
        @param target DirectedPoint of robot target position
        @return [V1, V2, V3, V4] list of motor powers for each robot
        """
        vector_target = current - target

        # setup mecanum control params
        # angle to translate at, radians 0-2pi
        target_angle = math.radians(vector_target.theta) % (2 * math.pi)
        target_speed = 1 # speed robot moves at [-1, 1]
        target_rot_speed = 0 # how quickly to change robot orientation [-1, 1]

        # Compute motors, 1-4, with forward direction as specified:
        pi4 = math.pi / 4.0
        V1 = target_speed * math.sin(target_angle + pi4) + target_rot_speed
        V2 = target_speed * math.cos(target_angle + pi4) - target_rot_speed
        V3 = target_speed * math.cos(target_angle + pi4) + target_rot_speed
        V4 = target_speed * math.sin(target_angle + pi4) - target_rot_speed





if __name__ == "__main__":
    robotcomm = RobotCommunication()
    robotcomm.connectToOffboard()
    while(1):
        msg = robotcomm.listenForMessage()
        if msg is None:
            continue
        else:
            print msg
            break
