'''
Main offboard controller.
Runs fixed-rate loop that pulls data from subsystems
'''
import sys
import time
import atexit

from IPython import embed

import subsystems
from utils.geometry import DirectedPoint, Waypoint
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

        data = self.sys_localization.getLocalizationData()
        blue_tf = data.robots[cst.TAG_ROBOT1]

        # Plan Paths
        blueStart = DirectedPoint(0, 0, 0)
        badStart = DirectedPoint(0, 0, 0)
        try:
            blueStart = data.robots[cst.TAG_ROBOT1]
        except:
            print("blue position not found on setup()")
        try:
            badStart = data.robots[cst.TAG_ROBOT2]
        except:
            print("bad position not found on setup()")

        (self.bluePath, self.badPath) = self.sys_planner.planTrajectories(blueStart, badStat)
        #self.sys_ui.drawDistribution(self.bluePath, self.badPath)

    def loop(self):
        '''
        Main offboard controller loop
        '''
        print('offboard main loop')
        # LocomotionData to stop robot and disable writing implement
        stop_locomotion = LocomotionData(
            tf_robot=DirectedPoint(0,0,0),
            tf_target=DirectedPoint(0,0,0),
            write_status=cst.WRITE_DISABLE,
            stop_status=cst.ROBOT_STOP)

        ######## DEBUG WAYPOINT TESTING ##########
        wp1 = Waypoint(DirectedPoint(5, 5, 0), cst.WRITE_DISABLE)
        wp2 = Waypoint(DirectedPoint(5, 8, 0), cst.WRITE_ENABLE)
        test_target = wp1.target
        test_write = wp1.write_status
        debug_waypoint = 0
        ########## END DEBUG WAYPOINT TESTING #########

        stop_status = cst.ROBOT_MOVE
        write_status = cst.WRITE_DISABLE
        
        path_index = 0

        blue_target = self.bluePath[path_index].target
        write_status = self.bluePath[path_index].write_status

        while True:

            ########## DEBUG WAYPOINT TESTING ############
            # # Test to move to 2 waypoints and write when moving to wp2
            # try:
            #     # Get localization data
            #     data = self.sys_localization.getLocalizationData()
            #     blue_tf = data.robots[cst.TAG_ROBOT1]
            #     # waypoint testing
            #     # if at waypoint 1, use waypoint 2
            #     if debug_waypoint is 0 and blue_tf.dist(wp1.target) < cst.STOP_DIST:
            #         print("At waypoint 1")  
            #         print(blue_tf,"| ",blue_tf.dist(wp1.target))
            #         print(blue_tf,"| ", blue_tf.dist(wp2.target))
            #         debug_waypoint = 1
            #         test_write = wp2.write_status
            #         test_target = wp2.target

            #         # send stop command
            #         self.sys_comm.generateMessage(
            #             robot_id=cst.BLUE_ID, locomotion=stop_locomotion,
            #             error=None)
            #         self.sys_comm.sendTCPMessages()
            #         time.sleep(1)

            #     if debug_waypoint is 1 and blue_tf.dist(wp2.target) < cst.STOP_DIST:
            #         print("At waypoint 2, stopping")
            #         print(blue_tf,"| ",blue_tf.dist(wp1.target))
            #         print(blue_tf,"| ", blue_tf.dist(wp2.target))
            #         stop_status = 1
            #         debug_waypoint = 2

            #     # Theta correction
            #     test_target.theta = data.corners[cst.TAG_TOP_RIGHT].theta                      
                
            #     test_locomotion = LocomotionData(
            #         tf_robot=blue_tf, 
            #         tf_target=test_target,
            #         write_status=cst.WRITE_DISABLE,
            #         stop_status=stop_status)

            #     self.sys_comm.generateMessage(
            #         robot_id=cst.BLUE_ID, locomotion=test_locomotion, 
            #         error=None)

            #     self.sys_comm.sendTCPMessages()
            #     continue

            # except:
            #     continue
            ################# END WAYPOINT WITH WRITING TEST #########

            try:
                # print("=========== new iteration ============")
                data = self.sys_localization.getLocalizationData()
                blue_tf = data.robots[cst.TAG_ROBOT1]
                # print("blue pos: ", str(blue_tf))

                # If at the waypoint, set next waypoint
                if blue_tf.dist(blue_target) < cst.STOP_DIST:
                    print("Waypoint", path_index, " reached:", 
                            str(self.bluePath[path_index]))

                    # send temporary stop command
                    self.sys_comm.generateMessage(
                        robot_id=cst.BLUE_ID, locomotion=test_locomotion, 
                        error=None)
                    self.sys_comm.sendTCPMessages()

                    # Set next waypoint
                    path_index += 1
                    # CHECK LEN-1 BECAUSE ALWAYS RETURNS TO CORNER
                    if path_index >= self.bluePath.length - 1:
                        stop_status = 1
                        print("FINAL WAYPOINT REACHED")


                    blue_target = self.bluePath[path_index]

                    time.sleep(1)

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

            time.sleep(0.01)


    def close(self):
        # Send message to stop robot
        print("Shutting down...")
        stop_locomotion = LocomotionData(
            DirectedPoint(0, 0, 0),
            DirectedPoint(0, 0, 0),
            cst.WRITE_DISABLE,
            cst.ROBOT_STOP)
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

    controller = OffboardController(robot_ip=blueRobotIP, drawing_name='test15')
    controller.robotSetup()
    controller.loop()
