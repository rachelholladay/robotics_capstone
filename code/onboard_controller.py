'''
Main onboard controller.
'''
import sys
from messages import robot_commands_pb2
from onboard.robot_communication import RobotCommunication


class OnboardController(object):
    def __init__(self, robot_ip):
        '''
        Instantiates main subsystems based on input parameters
        '''
        self.robot_ip = robot_ip




    def _test(self):
        pass

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
