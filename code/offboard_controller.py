'''
Main offboard controller.
Runs fixed-rate loop that pulls data from subsystems
'''
import sys
import time

from IPython import embed

import subsystems
from utils import constants as cst

class OffboardController(object):
    def __init__(self, robot_ip, drawing_number):
        '''
        Instantiates main subsystems based on input parameters
        '''
        self.robot_ip = robot_ip

        # self.sys_planner = subsystems.PlannerSystem()
        self.sys_localization = subsystems.LocalizationSystem(scaled_dims=[1,1])
        self.sys_locomotion = subsystems.LocomotionSystem()
        self.sys_comm = subsystems.CommunicationSystem()
        # self.sys_ui = subsystems.UISystem()

        # data = self.sys_ui.parseInputPaths('inputs/test{}'.format(drawing_number))
        # paths = self.sys_planner.planTrajectories(data)
        
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
        self.sys_localization.setup()
        self.sys_localization.begin_loop(verbose=0)

    def loop(self):
        '''
        Main offboard controller loop
        '''
        print 'offboard main loop'
        test_data = None
        t = 0
        while True:
            # if t is 0:
            #     test_data = [0, 0, 0, 1, 0]
            # elif t is 5:
            #     test_data = [0, 1, 1, 1, 0]
            # elif t is 10:
            #     test_data = [1, 1, 0, 0, 0]
            # elif t >= 20:
            #     test_data = [0, 0, 0, 0, 1]
            
            test_data = [0, 0, 1, 0, 0]
            self.sys_comm.generateMessage(0, None, None, test=test_data)
            self.sys_comm.sendTCPMessages()

            try:
                data = self.sys_localization.getLocalizationData()
                blue_pos = data.robots[cst.TAG_ROBOT1]
                print("blue pos: ", str(blue_pos))
                test_data = [blue_pos.x, blue_pos.y, 0.5, 0.5, 0]

                self.sys_comm.generateMessage(0, None, None, test=test_data)
                self.sys_comm.sendTCPMessages()
                print("Sent message")
            except:
                print("Failed to localize")

            time.sleep(0.5)
            t += 1


        # Original, untested loop
        # while True:
        #     if self.sys_planner.drawingComplete():
        #         break


        #     robot_messages = self.sys_comm.getTCPMessages()
        #     localization = self.sys_localization.getLocalization()

        #     paths = self.sys_planner.updatePaths(localization)

        #     locomotion_msg = self.sys_locomotion.generateCommand(localization, paths)
        #     writing_msg = self.sys_writing.generateCommand(localization, paths)
        #     error_msg = self.sys_comm.generateErrorCommand()

        #     self.sys_comm.sendMessage(locomotion_msg, writing_msg, error_msg)

        #     self.sys_ui.displayInfo(locomotion_msg, writing_msg, error_msg)


    def close(self):
        self.sys_localization.close()

    def _test(self):
        pass


if __name__ == "__main__":
    robotIPs = ['111.111.1.1', '222.222.2.2']
    localhost = ['localhost']
    testRobot = ['192.168.0.23']
    controller = OffboardController(robot_ip=testRobot, drawing_number=1)
    controller.robotSetup()
    controller.loop()



    # from messages import robot_commands_pb2
    # import socket

    # commsys = subsystems.CommunicationSystem()
    # commsys.connectToRobot('localhost', 0)
    # commsys.sendTCPMessages()

    # Localization test
    # loc = subsystems.LocalizationSystem(scaled_dims=[1,1])
    # loc.setup()
    # loc.begin_loop(verbose=0)
    # while(True):
    #     data = loc.getLocalizationData()
    #     if data is not None:
    #         embed()

    # serialized = cmd.SerializeToString()

    # commsys = CommunicationSystem()
    # address=('localhost', 5555)
    # buf_size = 1024
    # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # s.connect(address)
    # s.send(serialized)
    # while 1:
    # 	serialized = s.recv(buf_size)
    # 	data = cmd.ParseFromString(serialized)
    # 	if data is None:
    #         continue
    #     print 'received echo: '
    #     print data

    # s.close()

