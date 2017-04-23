'''
Main offboard controller.
Runs fixed-rate loop that pulls data from subsystems
'''
import sys
import time
import atexit

from IPython import embed

import subsystems
from utils.geometry import DirectedPoint
from utils.dataStorage import LocomotionData
from utils import constants as cst

class OffboardController(object):
    def __init__(self, robot_ip, drawing_name):
        '''
        Instantiates main subsystems based on input parameters
        '''
        self.robot_ip = robot_ip

        self.scaled_dims = [cst.TOP_BORDER - cst.BOTTOM_BORDER,
                            cst.RIGHT_BORDER - cst.LEFT_BORDER]

        # self.sys_planner = subsystems.PlannerSystem()
        self.sys_localization = subsystems.LocalizationSystem(
            scaled_dims=self.scaled_dims)
        self.sys_locomotion = subsystems.LocomotionSystem()
        self.sys_comm = subsystems.CommunicationSystem()
        self.sys_ui = subsystems.UISystem()
   
        data = self.sys_ui.parseInputPaths('inputs/{}'.format(drawing_name))
        self.sys_planner = subsystems.PlannerSystem(data)
   
        atexit.register(self.close)
        
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

        # Plan Paths
        #FIXME @NEIL insert real values
        badStart = DirectedPoint(0, 0, 0)
        blueStart = DirectedPoint(10, 10, 0)
        (self.bluePath, self.badPath) = self.sys_planner.planTrajectories(blueStart, badStat)
        #self.sys_ui.drawDistribution(self.bluePath, self.badPath)

    def loop(self):
        '''
        Main offboard controller loop
        '''
        print 'offboard main loop'
        # Debugging and waypoint testing
        stop_locomotion = LocomotionData(
            DirectedPoint(0,0,0),
            DirectedPoint(0,0,0),
            1)
        waypoint1 = DirectedPoint(5, 5, 0)
        waypoint2 = DirectedPoint(2.5, 2.5, 0)
        test_target = waypoint1
        debug_waypoint = 0


        stop_status = 0

        path_index = 1 # SKIPPING FIRST POINT B/C ALWAYS (0,0)
        blue_target = self.bluePath[path_index]

        while True:
            # #Simple test to move forward
            # test = LocomotionData(
            #     DirectedPoint(0, 0, 0),
            #     DirectedPoint(1, 0, 0),
            #     0)
            # self.sys_comm.generateMessage(robot_id=cst.BLUE_ID, 
            #     locomotion=test, error=None)
            # self.sys_comm.sendTCPMessages()
            # time.sleep(0.2)
            # continue
                      

            try:
                # print("=========== new iteration ============")
                data = self.sys_localization.getLocalizationData()
                blue_tf = data.robots[cst.TAG_ROBOT1]
                # print("blue pos: ", str(blue_tf))

                # waypoint testing
                # # if at waypoint 1, use waypoint 2
                # if debug_waypoint is 0 and blue_tf.dist(waypoint1) < 0.05:
                #     debug_waypoint = 1
                #     test_target = waypoint2
                #     print("At waypoint 1")
                # if debug_waypoint is 1 and blue_tf.dist(waypoint2) < 0.05:
                #     print("At waypoint 2, stopping")
                #     stop_status = 1
                #     debug_waypoint = 2

                # # Theta correction
                # test_target.theta = data.corners[cst.TAG_TOP_RIGHT].theta

                # If at the waypoint, set next waypoint
                if blue_tf.dist(blue_target) < 0.05:
                    print("WAYPOINT", path_index, " REACHED")
                    print("Waypoint: ", str(self.bluePath[path_index]))
                    path_index += 1
                    # CHECK LEN-1 BECAUSE ALWAYS RETURNS TO CORNER
                    if path_index >= self.bluePath.length - 1:
                        stop_status = 1
                        print("FINAL WAYPOINT REACHED")


                    blue_target = self.bluePath[path_index]

                # Theta correction
                blue_target.theta = data.corners[cst.TAG_TOP_RIGHT].theta

                blue_locomotion = LocomotionData(
                    blue_tf, 
                    blue_target,
                    stop_status)

                self.sys_comm.generateMessage(
                    robot_id=cst.BLUE_ID, locomotion=blue_locomotion, 
                    error=None)
                self.sys_comm.sendTCPMessages()

            except:
                pass
                # print("Failed to localize")

            time.sleep(0.01)


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
        # Send message to stop robot
        print("Shutting down...")
        stop_locomotion = LocomotionData(
            DirectedPoint(0, 0, 0),
            DirectedPoint(0, 0, 0),
            1)
        self.sys_comm.generateMessage(
            robot_id=cst.BLUE_ID, locomotion=stop_locomotion, 
            error=None)
        self.sys_comm.sendTCPMessages()
        self.sys_comm.closeTCPConnections()

        self.sys_localization.close()

    def _test(self):
        pass


if __name__ == "__main__":
    robotIPs = ['111.111.1.1', '222.222.2.2']
    localhost = ['localhost']
    blueRobotIP = ['192.168.0.23']

    controller = OffboardController(robot_ip=blueRobotIP, drawing_number='test15')
    controller.robotSetup()
    controller.loop()


    # from messages import robot_commands_pb2
    # import socket

    # commsys = subsystems.CommunicationSystem()
    # commsys.connectToRobot('localhost', 0)
    # commsys.sendTCPMessages()

    # Localization test
    # loc = subsystems.LocalizationSystem(scaled_dims=[1,1])
    # print("created")
    # loc.setup(1)
    # loc.begin_loop(verbose=1)
    # print("main localization started")
    # while(True):
    #         data = loc.getLocalizationData()
    #         time.sleep(0.5)



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
