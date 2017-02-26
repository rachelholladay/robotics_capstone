'''
Main onboard controller.
'''
import sys
from subsystems.communication import CommunicationSystem


class OnboardController(object):
    def __init__(self, robot_ip):
        '''
        Instantiates main subsystems based on input parameters
        '''
        self.robot_ip = robot_ip




    def _test(self):
        pass

if __name__ == "__main__":
    offboard_ip = 111.111.1.1

    controller = OffboardController(robot_ip=robotIPs)
    controller.robotSetup()
    # controller.loop()
