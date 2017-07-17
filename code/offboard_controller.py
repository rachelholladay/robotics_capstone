"""
Main offboard controller. This class gets run to operate the offboard 
subsystems and control the robots to successfully complete specified drawings.

Pulls information continually from other subsystems to send locomotion
information to the onboard robot controllers. Also shows debugging data
and system status during operation.
"""
from __future__ import print_function

import sys
import time

import subsystems
from utils import constants as cst
from utils.geometry import DirectedPoint, Waypoint
from utils.dataStorage import LocomotionData


class OffboardController(object):
    """
    This class represents the main control system for the robot system. During
    system operation, a single intance of this class is initialized and run
    to control worker robots. 

    The offboard controller processes localization and current robot state
    to plan and send commands via TCP connection to the onboard controllers
    on each individual robot.

    Sample operation of this class is as follows:
        controller = OffboardController(robot_ids=['robot_ip1', robot_ip2'],
                                        drawing_name='drawing_name_file')
        controller.robotSetup()
        controller.loop()
        controller.close()
    """
    def __init__(self, robot_ids, drawing_name):
        """
        Instantiates main subsystems based on input parameters.
        Initializes various subsystems and runs preprocessing and
            initial planning for robot motions. Note that as a part
            of offboard initialization, no attempt at connecting
            to the physical robots via TCP is made. This is done
            at runtime within the robotSetup() function.

        @param robot_ids The robot ID as specified by constants.py
            to denote which robots will be running.
        @param drawing_name String representing the input drawing
            to be planned and created by the robots.
        """
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

        # LocomotionData struct to stop robot and disable writing
        # implement
        self.stop_locomotion = LocomotionData(
            tf_robot=DirectedPoint(0, 0, 0),
            tf_target=DirectedPoint(0, 0, 0),
            write_status=cst.WRITE_DISABLE,
            stop_status=cst.ROBOT_STOP)

        self._debug = 1

    def robotSetup(self):
        """
        Sets up communication links with robot agents, and uses
        intitial localization data to refine robot path planning.

        Robot position and orientation is used to append waypoints
        to the planner to allow the robots to reach their intended
        targets. This feature allows the robots to be placed anywhere
        in the drawing space, but still have the ability to move to
        their first waypoint and complete the drawing.
        """
        for i in range(0, len(self.robot_ids)):
            success = self.sys_comm.connectToRobot(self.robot_ids[i])
            if not success:
                print("FAILED TO CONNECT TO ROBOT")
                sys.exit(1)

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

        if self._debug:
            print("blue start", str(blueStart))
            print("bad start", str(badStart))

        (self.bluePath, self.badPath) = self.sys_planner.planTrajectories(
            blueStart, badStart)
        if len(self.bluePath.path) is 0:
            self.bluePath.path.append(Waypoint(blueStart, cst.WRITE_DISABLE))
        if len(self.badPath.path) is 0:
            self.badPath.path.append(Waypoint(badStart, cst.WRITE_DISABLE))

        self.paths = [self.bluePath, self.badPath]
        if self._debug:
            print("blue path")
            print(self.bluePath)
            print("bad path")
            print(self.badPath)

        # self.sys_ui.drawDistribution(self.bluePath, self.badPath)

    def loop(self):
        """
        Main offboard controller loop. Continually polls for incoming
        localization data, then processes into a goal position for
        the robot. Errors and other safety-related information is
        incorporated into robot positional targeting. This includes
        robot-robot collision avoidance, and well as bounds awareness.
        """
        # Setup robot parameters for constructing messages
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
            self.paths[cst.BAD_ID][self.path_index[cst.BAD_ID]].target]
        self.write_status = [
            self.paths[cst.BLUE_ID][self.path_index[cst.BLUE_ID]].write_status,
            self.paths[cst.BAD_ID][self.path_index[cst.BAD_ID]].write_status]

        # Send initial message to stop motion, indicating setup completion
        if self._debug:
            print("loop setup: disable writing tool")
        for rid in self.robot_ids:
            self.sys_comm.sendTCPMessage(rid, self.stop_locomotion)
        time.sleep(0.5)

        # Run actual loop
        while True:
            data = self.sys_localization.getLocalizationData()

            if self.completed == [True, True]:
                if self._debug:
                    print("Drawing complete")

                for rid in self.robot_ids:
                    self.sys_comm.sendTCPMessage(rid, self.stop_locomotion)
                return

            # Basic collision code
            blue_tf = data.robots[cst.TAG_ROBOT1]
            bad_tf = data.robots[cst.TAG_ROBOT2]

            # Both robots must be discovered on the drawing surface
            # to acucurately check collision. If either is not found,
            # assume an error state and stop robot locomotion for
            # both robots.
            if not (blue_tf.valid and bad_tf.valid):
                for rid in self.robot_ids:
                    self.sys_comm.sendTCPMessage(rid,
                                                 self.stop_locomotion)
                continue

            # check collision buffer threshold for actual collision
            if blue_tf.dist(bad_tf) < cst.COLLISION_BUFFER:
                if self._debug:
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
        Given the target location for the robot, uses localization
        data to compute the next goal position and sends the data
        to the robot's onboard controller.

        Current robot positional information and the expected target
        information is combined to determine if the robot is within
        predetermined range of its next waypoint. If a waypoint is
        reached, the next waypoint in the path is selected and used
        as a goal.

        For the project demo, the writing surface had a small gap
        underneath approximately halfway across. This caused uneven
        writing and often the robots became stuck when attempting
        to write while crossing the gap. To account for this, this
        function also lifts the writing implement when the robot would
        otherwise be attempting to write while crossing the gap.

        Finalized localization and targeting data is sent via TCP
        connection to the onboard controller for processsing.

        @param robot_id ID of the robot to control
        @param localization_data Most recent localization information
            for both robots.
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
            if self._debug:
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

                if self._debug:
                    print("FINAL WAYPOINT REACHED ROBOT", robot_id)

                # send stop
                self.sys_comm.sendTCPMessage(robot_id,
                                             self.stop_locomotion)
                time.sleep(0.5)
                return

            # Retrieve valid target and writing status information
            self.targets[robot_id] = \
                self.paths[robot_id][self.path_index[robot_id]].target
            self.write_status[robot_id] = \
                self.paths[robot_id][self.path_index[robot_id]].write_status

            # Send stop command, then raise writing implement after stopping
            stop_wp = self.stop_locomotion
            stop_wp.write_status = self.write_status[robot_id]

            self.sys_comm.sendTCPMessage(robot_id,
                                         self.stop_locomotion)
            time.sleep(1)
            self.sys_comm.sendTCPMessage(robot_id, stop_wp)
            time.sleep(0.5)

        # Retrieve theta correction information - pushes robot locomotion
        # controller to always maintain heading with the fixed top right tag.
        self.targets[robot_id].theta = data.corners[cst.TAG_TOP_RIGHT].theta

        robot_locomotion = LocomotionData(
            tf_robot=robot_tf,
            tf_target=self.targets[robot_id],
            write_status=self.write_status[robot_id],
            stop_status=self.stop_status[robot_id])

        # Disable drawing if within gap in floor - workaround for uneven demo
        # space. Only need to disable if tool status should be enabled
        if cst.GAP_ENABLED and \
           robot_locomotion.write_status is cst.WRITE_ENABLE:
            if abs(robot_tf.y - cst.GAP_LOCATION) < cst.GAP_BUFFER:
                robot_locomotion.write_status = cst.WRITE_DISABLE

        self.sys_comm.sendTCPMessage(robot_id, robot_locomotion)

    def close(self):
        # Send message to stop robot
        if self._debug:
            print("Shutting down...")
        for rid in self.robot_ids:
            self.sys_comm.sendTCPMessage(rid, self.stop_locomotion)

        # End subsystem operation
        time.sleep(2)
        self.sys_comm.closeTCPConnections()
        self.sys_localization.close()


if __name__ == "__main__":

    blueID = [cst.BLUE_ID]
    badID = [cst.BAD_ID]
    robotIDs = [cst.BLUE_ID, cst.BAD_ID]

    # Demo and testing filenames and calibration
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

    # Setup and run controller until drawing completion or early exit by user
    controller = OffboardController(robot_ids=robotIDs,
                                    drawing_name=demo_cu)
    controller.robotSetup()

    try:
        controller.loop()
        controller.close()
    except KeyboardInterrupt:
        controller.close()
