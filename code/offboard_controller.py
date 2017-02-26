'''
Main offboard controller.
Runs fixed-rate loop that pulls data from subsystems
'''
import sys
from subsystems.communication import CommunicationSystem
from subsystems.localization import LocalizationSystem
from subsystems.locomotion import LocomotionSystem
from subsystems.planner import PlannerSystem
from subsystems.ui import UISystem

class OffboardController(object):
    def __init__(self, robot_ip):
        '''
        Instantiates main subsystems based on input parameters
        '''
        self.robot_ip = robot_ip

        self.sys_planner = PlannerSystem()
        self.sys_localization = LocalizationSystem()
        self.sys_locomotion = LocomotionSystem()
        self.sys_comm = CommunicationSystem()
        self.sys_ui = UISystem()


    def processInputData(self, data):
        '''
        Receives input image and runs planner
        '''
        paths = self.sys_planner.planTrajectories(data)

    def robotSetup(self):
        '''
        Sets up communication links with robot agents.
        Setup step for drawing loop
        '''
        for i in xrange(0, len(self.robot_ip)):
            success = self.sys_comm.connectToRobot(i, self.robot_ip[i])
            if not success:
                print 'FAILED TO CONNECT TO ROBOT'
                sys.exit(1)


        # TODO connect to camera, ensure valid connection

        # TODO start localization, planner and UI in threads

    def loop(self):
        '''
        Main offboard controller loop
        '''
        while True:
            if self.sys_planner.drawingComplete():
                break


            robot_messages = self.sys_comm.getTCPMessages()
            localization = self.sys_localization.getPositions()

            paths = self.sys_planner.updatePaths(localization)

            locomotion_msg = self.sys_locomotion.generateCommand(localization, paths)
            writing_msg = self.sys_writing.generateCommand(localization, paths)
            error_msg = self.sys_comm.generateErrorCommand()

            self.sys_comm.sendMessage(locomotion_msg, writing_msg, error_msg)

            self.sys_ui.displayInfo(locomotion_msg, writing_msg, error_msg)


    def _test(self):
        pass


if __name__ == "__main__":
    robotIPs = ['111.111.1.1', '222.222.2.2']
    test_proto = test_pb2.test_msg()
    test_proto.name = 'ASDF'

    controller = OffboardController(robot_ip=robotIPs)
    controller.robotSetup()
    # controller.loop()
