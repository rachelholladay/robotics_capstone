'''
Main offboard controller.
Runs fixed-rate loop that pulls data from subsystems
'''
from __future__ import print_function

import sys
import signal
import time
import cProfile

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


    def robotSetup(self):
        '''
        Sets up communication links with robot agents.
        Setup step for drawing loop
        '''
        for i in xrange(0, len(self.robot_ids)):
            success = self.sys_comm.connectToRobot(self.robot_ids[i])
            if not success:
                print("FAILED TO CONNECT TO ROBOT")
                sys.exit(1)

        # TODO start localization, planner and UI in threads
        self.sys_localization.setup()
        self.sys_localization.begin_loop(verbose=0)
        time.sleep(1)

        data = self.sys_localization.getLocalizationData()

        # Plan Paths based on initial localization
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
        """
        Main offboard controller loop
        """
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

        # Send initial message to stop motion, setup completion
        print("disable writing tool")
        for rid in self.robot_ids:
            self.sys_comm.sendTCPMessage(rid, self.stop_locomotion)
        time.sleep(0.5)

        # Run actual loop
        while True:
            data = self.sys_localization.getLocalizationData()

            if self.completed == [True, True]:
                print("Drawing complete")
                for rid in self.robot_ids:
                    self.sys_comm.sendTCPMessage(rid, self.stop_locomotion)
                return

            # Basic collision code
            # TODO actuate bad if not in collision range
            blue_tf = data.robots[cst.TAG_ROBOT1]
            bad_tf = data.robots[cst.TAG_ROBOT2]

            # ensure both tags are found before checking collision
            # if both robots not found, stop execution
            if not (blue_tf.valid and bad_tf.valid):
                for rid in self.robot_ids:
                    self.sys_comm.sendTCPMessage(rid,
                        self.stop_locomotion)
                continue

            # check collision buffer threshold
            if blue_tf.dist(bad_tf) < cst.COLLISION_BUFFER:
                print("IN COLLISION BY", blue_tf.dist(bad_tf))
                self.sys_comm.sendTCPMessage(cst.BAD_ID,
                    self.stop_locomotion)
                
                if self.completed[cst.BLUE_ID] is not True:
                    self.commandRobot(cst.BLUE_ID, data)
            else:
                # Only command robot if it is not done drawing
                if self.completed[cst.BAD_ID] is not True:
                    self.commandRobot(cst.BAD_ID, data)
                if self.completed[cst.BLUE_ID] is not True:
                    self.commandRobot(cst.BLUE_ID, data)

            time.sleep(0.1)

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

        # If at the waypoint, set next waypoint
        if robot_tf.distsq(self.targets[robot_id]) < cst.STOP_DIST_SQ:
            print("-----------Waypoint reached (Robot", 
                    robot_id, ") ------------")
            print("Waypoint", self.path_index[robot_id], " reached:", 
                    str(self.paths[robot_id][self.path_index[robot_id]]))            
            print(name, "tf: ", str(robot_tf))

            # Set next waypoint
            self.path_index[robot_id] += 1

            # Check if at the last waypoint, then stop
            if self.path_index[robot_id] >= self.paths[robot_id].length:
                self.stop_status[robot_id] = 1
                self.completed[robot_id] = True
                print("FINAL WAYPOINT REACHED ROBOT", robot_id)
                # send stop

                self.sys_comm.sendTCPMessage(robot_id, 
                    self.stop_locomotion)
                time.sleep(0.5)
                return

            self.targets[robot_id] = self.paths[robot_id][self.path_index[robot_id]].target
            self.write_status[robot_id] = self.paths[robot_id][self.path_index[robot_id]].write_status
            
            # Send stop command, then raise writing implement up after
            # stopping
            stop_wp = self.stop_locomotion
            stop_wp.write_status = self.write_status[robot_id]

            self.sys_comm.sendTCPMessage(robot_id,
                self.stop_locomotion)
            time.sleep(1)
            self.sys_comm.sendTCPMessage(robot_id, stop_wp)
            time.sleep(0.5)


        # Theta correction
        self.targets[robot_id].theta = data.corners[cst.TAG_TOP_RIGHT].theta

        robot_locomotion = LocomotionData(
            tf_robot=robot_tf, 
            tf_target=self.targets[robot_id],
            write_status=self.write_status[robot_id],
            stop_status=self.stop_status[robot_id])

        # Disable drawing if within gap in floor - workaround for uneven demo
        # space. Only need to disable if tool status should be enabled
        if cst.GAP_ENABLED and robot_locomotion.write_status is cst.WRITE_ENABLE:
            if abs(robot_tf.y - cst.GAP_LOCATION) < cst.GAP_BUFFER:
                # print("HACK ENABLED", str(robot_tf))
                robot_locomotion.write_status = cst.WRITE_DISABLE

        self.sys_comm.sendTCPMessage(robot_id, robot_locomotion)


    def close(self):
        # Send message to stop robot
        print("Shutting down...")        
        for rid in self.robot_ids:
            self.sys_comm.sendTCPMessage(rid, self.stop_locomotion)

        time.sleep(2)
        self.sys_comm.closeTCPConnections()
        self.sys_localization.close()




if __name__ == "__main__":


    blueID = [cst.BLUE_ID]
    badID = [cst.BAD_ID]
    robotIDs = [cst.BLUE_ID, cst.BAD_ID]

    oneLine = 'oneLine'
    twoLines = 'twoLines'
    centerLine = 'centerLine'
    centerShortLine = 'centerShortLine'
    shortLine = 'shortLine'
    threeShortLines = 'threeShortLines'
    connectedLines = 'connectedLines'
    debug = 'debug'
    twoBoxes = 'twoBoxes'
    horizLine = 'horizLine'
    x = 'x'
    cu = 'cu0'

    # start blue in bottom center ~(5,2)
    # start bad ~bottom left ~(2,3)
    demo_plus = 'demo_plus'

    # start blue top center offset left ~(3, 9)
    # start bad top right ~(9, 9)
    demo_cu = 'demo_cu'

    # start blue bottom left ~(2,2)
    # start bad top right ~(8,8)
    demo_twoLines = 'demo_twoLines'

    controller = OffboardController(robot_ids=robotIDs, 
        drawing_name=demo_plus)
    controller.robotSetup()
    # cProfile.run('controller.loop()')
    try:
        controller.loop()
        controller.close()
    except KeyboardInterrupt:
        controller.close()