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

        self.paths = [self.bluePath, self.bluePath]
        print("blue path")
        print(self.bluePath)
        print("bad path")
        print(self.badPath)
        #self.sys_ui.drawDistribution(self.bluePath, self.badPath)

    def loop(self):
        '''
        Main offboard controller loop
        '''
        print('offboard main loop')#

        # Setup robot parameters for loop
        # Drawing complete status - if a robot is not present, it can be
        # considered already 'finished'
        self.completed = [False, False]
        if cst.BLUE_ID not in self.robot_ids:
            self.completed[cst.BLUE_ID] = True
        if cst.BAD_ID not in self.robot_ids:
            self.completed[cst.BAD_ID] = True

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

        # Send initial message to stop motion, setup completion
        print("disable writing tool")
        for rid in self.robot_ids:
            self.sys_comm.generateMessage(
                robot_id=rid, locomotion=cmd_disable,
                error=None)
            self.sys_comm.sendTCPMessages()
        time.sleep(1)

        ### FIXME TODO DELETE THIS writing tool test
        # print("enable writing tool")
        # cmd_disable.write_status = cst.WRITE_ENABLE
        # self.sys_comm.generateMessage(
        #     robot_id=rid, locomotion=cmd_disable,
        #     error=None)
        # self.sys_comm.sendTCPMessages()
        # time.sleep(1)

        # print("disable writing tool")
        # for rid in self.robot_ids:
        #     self.sys_comm.generateMessage(
        #         robot_id=rid, locomotion=cmd_disable,
        #         error=None)
        #     self.sys_comm.sendTCPMessages()
        # time.sleep(1)
        # return
        ### END WRITING TOOL TEST

        # Run actual loop
        while True:
                data = self.sys_localization.getLocalizationData()

                if self.completed == [True, True]:
                    print("Drawing complete")
                    for rid in self.robot_ids:
                        cmd_disable.stop_status = cst.ROBOT_STOP
                        self.sys_comm.generateMessage(
                            robot_id=rid, locomotion=cmd_disable,
                            error=None)
                        self.sys_comm.sendTCPMessages()
                    return


                # Robot-to-robot commands
                # get blue tf, bad tf from localization data above
                # if bluetf.dist(bad_tf) < collision_buffer:
                #   send pause to bad tf
                # process blue tf as normal

                # Setup individual robot commands
                for rid in self.robot_ids:
                    if self.completed[rid] is True:
                        continue
                    self.commandRobot(rid, data)
                    # try:
                    #     if self.completed[rid] is True:
                    #         continue
                    #     self.commandRobot(rid, data)
                    # except:
                    #     pass



    def commandRobot(self, robot_id, localization_data):
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

        data = localization_data
        robot_tf = data.robots[tag]


        # print(name, str(robot_tf),"| ",robot_tf.dist(self.targets[robot_id]))
        # print(str(robot_tf), str(self.targets[robot_id]))
        # print(str(robot_tf), "| dist2target", robot_tf.dist(self.targets[robot_id]))

        # If at the waypoint, set next waypoint
        if robot_tf.dist(self.targets[robot_id]) < cst.STOP_DIST:
            print("-----------waypoint reached------------")
            print("Waypoint", self.path_index[robot_id], " reached:", 
                    str(self.paths[robot_id][self.path_index[robot_id]]))
            print(name, "tf: ", str(robot_tf))

            # Set next waypoint
            self.path_index[robot_id] += 1

            # Check if at the last waypoint, then stop
            if self.path_index[robot_id] >= self.paths[robot_id].length:
                self.stop_status[robot_id] = 1
                self.completed[robot_id] = True
                print("FINAL WAYPOINT REACHED")
                # send stop
                stop_wp = self.stop_locomotion
                stop_wp.stop_status = cst.ROBOT_STOP
                stop_wp.write_status = cst.WRITE_DISABLE
                self.sys_comm.generateMessage(
                    robot_id=robot_id, locomotion=stop_wp, 
                    error=None)
                self.sys_comm.sendTCPMessages()
                time.sleep(1)
                return

            self.targets[robot_id] = self.paths[robot_id][self.path_index[robot_id]].target
            self.write_status[robot_id] = self.paths[robot_id][self.path_index[robot_id]].write_status
            
            # send temporary stop command and also actuate writing
            # tool to new position
            stop_wp = self.stop_locomotion
            stop_wp.write_status = self.write_status[robot_id]
            self.sys_comm.generateMessage(
                robot_id=robot_id, locomotion=stop_wp, 
                error=None)
            self.sys_comm.sendTCPMessages()
            time.sleep(1)

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
        
        self.stop_locomotion.stop_status = cst.ROBOT_STOP
        for rid in self.robot_ids:
            self.sys_comm.generateMessage(
                robot_id=rid, locomotion=self.stop_locomotion, 
                error=None)
        self.sys_comm.sendTCPMessages()
        time.sleep(2)
        self.sys_comm.closeTCPConnections()
        self.sys_localization.close()

    def _test(self):
        pass


if __name__ == "__main__":

    blueID = [cst.BLUE_ID]
    badID = [cst.BAD_ID]
    robotIDs = [cst.BLUE_ID, cst.BAD_ID]

    oneLine = 'oneLine'
    centerLine = 'centerLine'
    shortLine = 'shortLine'

    controller = OffboardController(robot_ids=blueID, drawing_name=shortLine)
    controller.robotSetup()
    controller.loop()
    controller.close()