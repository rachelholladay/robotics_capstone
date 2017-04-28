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
    def __init__(self, robot_ids, drawing_name):
        '''
        Instantiates main subsystems based on input parameters
        '''
        self.robot_ids = robot_ids

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
   
        # LocomotionData to stop robot and disable writing implement
        self.stop_locomotion = LocomotionData(
            tf_robot=DirectedPoint(0,0,0),
            tf_target=DirectedPoint(0,0,0),
            write_status=cst.WRITE_DISABLE,
            stop_status=cst.ROBOT_STOP)

        atexit.register(self.close)
        
    def robotSetup(self):
        '''
        Sets up communication links with robot agents.
        Setup step for drawing loop
        '''
        for i in xrange(0, len(self.robot_ids)):
            success = self.sys_comm.connectToRobot(self.robot_ids[i])
            if not success:
                print 'FAILED TO CONNECT TO ROBOT'
                sys.exit(1)


        # TODO start localization, planner and UI in threads
        self.sys_localization.setup()
        self.sys_localization.begin_loop(verbose=0)
        time.sleep(1)
        data = self.sys_localization.getLocalizationData()
        blue_tf = data.robots[cst.TAG_ROBOT1]

        # Plan Paths
        blueStart = DirectedPoint(0, 0, 0)
        badStart = DirectedPoint(0, 0, 0)
        if data.robots[cst.TAG_ROBOT1].valid is True:
            blueStart = data.robots[cst.TAG_ROBOT1]
        else:
            print("blue position not found on setup()")
        if data.robots[cst.TAG_ROBOT2].valid is True:
            badStart = data.robots[cst.TAG_ROBOT2]
        else:
            print("bad position not found on setup()")
        print("blue start", str(blueStart))
        print("bad start", str(badStart))

        (self.bluePath, self.badPath) = self.sys_planner.planTrajectories(
            blueStart, badStart)
        if len(self.bluePath.path) is 0:
            self.bluePath.path.append(Waypoint(blueStart, cst.WRITE_DISABLE))
        if len(self.badPath.path) is 0:
            self.badPath.path.append(Waypoint(badStart, cst.WRITE_DISABLE))

        self.paths = [self.bluePath, self.badPath]
        print("blue path")
        print(self.bluePath)
        print("bad path")
        print(self.badPath)
        #self.sys_ui.drawDistribution(self.bluePath, self.badPath)

    def loop(self):
        '''
        Main offboard controller loop
        '''
        print('offboard main loop')

        ######## DEBUG WAYPOINT TESTING ##########
        wp1 = Waypoint(DirectedPoint(5, 5, 0), cst.WRITE_DISABLE)
        wp2 = Waypoint(DirectedPoint(5, 8, 0), cst.WRITE_ENABLE)
        test_target = wp1.target
        test_write = wp1.write_status
        debug_waypoint = 0
        ########## END DEBUG WAYPOINT TESTING #########

        # Setup robot parameters for loop
        self.stop_status = [cst.ROBOT_MOVE, cst.ROBOT_MOVE]
        self.write_status = [cst.WRITE_DISABLE, cst.WRITE_DISABLE]
        
        self.path_index = [0, 0]

        self.targets = [
                    self.paths[cst.BLUE_ID][self.path_index[cst.BLUE_ID]].target, 
                    self.paths[cst.BAD_ID][self.path_index[cst.BAD_ID]].target
                    ]
        self.write_status = [
                    self.paths[cst.BLUE_ID][self.path_index[cst.BLUE_ID]].write_status,
                    self.paths[cst.BAD_ID][self.path_index[cst.BAD_ID]].write_status
                    ]

        # send initial command to disable writing implement
        cmd_disable = self.stop_locomotion
        cmd_disable.stop_status = cst.ROBOT_MOVE
        cmd_disable.write_status = cst.WRITE_DISABLE

        for rid in self.robot_ids:
            self.sys_comm.generateMessage(
                robot_id=rid, locomotion=cmd_disable,
                error=None)
            self.sys_comm.sendTCPMessages()
        time.sleep(1)

        
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
            #             robot_id=cst.BLUE_ID, locomotion=self.stop_locomotion,
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
                self.commandRobot(cst.BLUE_ID)
                # for rid in self.robot_ids:
                #     self.commandRobot(rid)
                
                # # print("=========== new iteration ============")
                # data = self.sys_localization.getLocalizationData()
                # blue_tf = data.robots[cst.TAG_ROBOT1]
                # # print("blue pos: ", str(blue_tf))

                # # If at the waypoint, set next waypoint
                # if blue_tf.dist(blue_target) < cst.STOP_DIST:
                #     print("Waypoint", path_index, " reached:", 
                #             str(self.bluePath[path_index]))
                #     print("Blue tf: ", str(blue_tf))

                #     # Set next waypoint
                #     path_index += 1

                #     # Check if at the last waypoint, then stop
                #     if path_index >= self.bluePath.length:
                #         stop_status = 1
                #         print("FINAL WAYPOINT REACHED")

                #     blue_target = self.bluePath[path_index].target
                #     write_status = self.bluePath[path_index].write_status
                    
                #     # send temporary stop command and also actuate writing
                #     # tool to new position
                #     stop_wp = self.stop_locomotion
                #     stop_wp.write_status = write_status
                #     self.sys_comm.generateMessage(
                #         robot_id=cst.BLUE_ID, locomotion=stop_wp, 
                #         error=None)
                #     self.sys_comm.sendTCPMessages()
                #     time.sleep(1)

                # # print(str(blue_tf),"| ",blue_tf.dist(blue_target))
                    
                # # Theta correction
                # blue_target.theta = data.corners[cst.TAG_TOP_RIGHT].theta

                # blue_locomotion = LocomotionData(
                #     tf_robot=blue_tf, 
                #     tf_target=blue_target,
                #     write_status=write_status,
                #     stop_status=stop_status)

                # self.sys_comm.generateMessage(
                #     robot_id=cst.BLUE_ID, locomotion=blue_locomotion, 
                #     error=None)
                # self.sys_comm.sendTCPMessages()

            except:
                pass



    def commandRobot(self, robot_id):
        """
        Pulls localization for specified robot and sends message
        """
        tag = None
        name = ''
        if robot_id is cst.BLUE_ID:
            tag = cst.TAG_ROBOT1
            name = 'Blue'
        elif robot_id is cst.BAD_ID:
            tag = cst.TAG_ROBOT2
            name = 'Bad'

        data = self.sys_localization.getLocalizationData()
        robot_tf = data.robots[tag]

        # If at the waypoint, set next waypoint
        if robot_tf.dist(self.targets[robot_id]) < cst.STOP_DIST:
            print("Waypoint", path_index[robot_id], " reached:", 
                    str(self.paths[robot_id][path_index]))
            print(name, "tf: ", str(robot_tf))

            # Set next waypoint
            path_index[robot_id] += 1

            # Check if at the last waypoint, then stop
            if path_index[robot_id] >= self.paths[robot_id].length:
                self.stop_status[robot_id] = 1
                print("FINAL WAYPOINT REACHED")

            self.targets[robot_id] = self.paths[robot_id][path_index].target
            write_status = self.paths[robot_id][path_index].write_status
            
            # send temporary stop command and also actuate writing
            # tool to new position
            stop_wp = self.stop_locomotion
            stop_wp.write_status = self.write_status[robot_id]
            self.sys_comm.generateMessage(
                robot_id=robot_id, locomotion=stop_wp, 
                error=None)
            self.sys_comm.sendTCPMessages()
            time.sleep(1)

        # print(name, str(robot_tf),"| ",robot_tf.dist(self.targets[robot_id]))
        print(str(robot_tf), str(self.targets[robot_id]))
        # Theta correction
        self.targets[robot_id].theta = data.corners[cst.TAG_TOP_RIGHT].theta

        robot_locomotion = LocomotionData(
            tf_robot=robot_tf, 
            tf_target=self.targets[robot_id],
            write_status=cst.WRITE_DISABLE,#self.write_status[robot_id],
            stop_status=self.stop_status[robot_id])

        self.sys_comm.generateMessage(
            robot_id=robot_id, locomotion=robot_locomotion, 
            error=None)
        self.sys_comm.sendTCPMessages()


    def close(self):
        # Send message to stop robot
        print("Shutting down...")
        
        for rid in self.robot_ids:
            self.sys_comm.generateMessage(
                robot_id=rid, locomotion=self.stop_locomotion, 
                error=None)
        self.sys_comm.sendTCPMessages()
        self.sys_comm.closeTCPConnections()

        self.sys_localization.close()

    def _test(self):
        pass


if __name__ == "__main__":
    robotIPs = [cst.BLUE_IP, cst.BAD_IP]
    localhost = ['localhost']
    blueRobotIP = [cst.BLUE_IP]
    badRobotIP = [cst.BAD_IP]

    blueID = [cst.BLUE_ID]
    badID = [cst.BAD_ID]
    robotIDs = [cst.BLUE_ID, cst.BAD_ID]

    oneLine = 'oneLine'
    centerLine = 'centerLine'

    controller = OffboardController(robot_ids=blueID, drawing_name=oneLine)
    controller.robotSetup()
    controller.loop()
