'''
Main onboard controller.
'''
from __future__ import print_function

import sys
import math
import time
import atexit 
import threading 

import numpy as np

from messages import robot_commands_pb2
from onboard.robot_communication import RobotCommunication
from onboard.motors import Motors
from utils.geometry import DirectedPoint

import utils.constants as cst

class OnboardController(object):
    def __init__(self, robot_ip):
        '''
        Instantiates main subsystems based on input parameters
        '''
        self.robot_ip = robot_ip
        self.comm = RobotCommunication()
        self.motors = Motors()
        self.motors.stopMotors()

        self.message_timer = 0
        self._watchdog_thread = None
        self._stop_thread = False

        atexit.register(self.close)

    def setup(self):
        self.motors.disableWrite()

        self.comm.connectToOffboard()
        time.sleep(1)

        # start message watchdog timer to ensure motion does not occur without
        # receiving offboard messages
        self._watchdog_thread = threading.Thread(name='_message_watchdog',
                             target=self._message_watchdog) 
        self._watchdog_thread.start()      

    def loop(self):
        """
        Main offboard control loop. Listens for protobuf messages from the
        offboard system, parses and runs appropriate motor command.
        """
        print("onboard main loop")
        self.message_timer = time.time()

        prev_robot = None
        prev_target = None

        while(1):

            msg = self.comm.listenForMessage()
            if msg is None:
                continue
            else:
                if msg.stop_status is 1:
                    self.motors.stopMotors()
                    if msg.write_status is cst.WRITE_DISABLE:
                        self.motors.disableWrite()
                else:
                    print("======= new message",time.time()," =======")

                    self.message_timer = time.time()

                    # Specified target from offboard system
                    robot_pos = DirectedPoint(
                        msg.robot_x, msg.robot_y, theta=msg.robot_th)
                    target_pos = DirectedPoint(
                        msg.target_x, msg.target_y, theta=msg.target_th)
                    write_status = msg.write_status


                    # ###### Controller to fix accuracy #####
                    # # TODO make function, directional adjustment not necessary
                    # # Use previous-message motion vector and actual robot
                    # # position relative to previous vector to adjust target
                    # # to account for any motion error
                    # # Error vector based on 
                    # # http://stackoverflow.com/questions/5227373/minimal-perpendicular-vector-between-a-point-and-a-line
                    # if prev_target is None or prev_robot is None:
                    #     # first message, set previous for next message
                    #     prev_robot = robot_pos
                    #     prev_target = target_pos
                    # else:
                    #     # Use robot's current position and find offset from
                    #     # ideal vector of prev_target - prev_robot
                    #     prev_direction = prev_target - prev_robot
                    #     prev_dir_mag = math.sqrt(prev_direction.x**2 + prev_direction.y**2)

                    #     prev_direction.x = prev_direction.x / prev_dir_mag
                    #     prev_direction.y = prev_direction.y / prev_dir_mag

                    #     error_vector = (prev_robot + \
                    #         ((robot_pos - prev_robot).dot(prev_direction)) * prev_direction) \
                    #         - robot_pos

                    #     # print("error vec:", error_vector)

                    #     # Update previous for next iteration
                    #     prev_robot = robot_pos
                    #     prev_target = target_pos

                    #     # Offset position by error to correct
                    #     # robot_pos = robot_pos + error_vector

                    # # Set writing status accordingly
                    # print("write status", write_status)
                    # if write_status is cst.WRITE_ENABLE:
                    #     print("Enable writing")
                    #     self.motors.enableWrite()
                    # else:
                    #     print("Disable writing")
                    #     self.motors.disableWrite()


                    print("Moving from", robot_pos, " to", target_pos)
                    motor_commands = self.getMotorCommands(robot_pos, target_pos)
                    self.moveMotors(motor_commands)

                # reset state
                msg = None
                

    def moveMotors(self, command):
        """
        Commands all motors using given command (i.e. DIR_UPLEFT).
        @param command Motor command to run
        """
        print("Moving:", command)
        for i in range(0, 4):
            self.motors.commandMotor(i, command[i])

    def moveMotorsTime(self, command, t=0.3):
        """
        Commands all motors using a given command (such as DIR_UPLEFT) for a time
        in seconds.
        @param command Motor command to run
        @param t Time in seconds to move for
        """
        print("Moving", command, " for", t, " seconds.")
        for i in range(0, 4):
            self.motors.commandMotor(i, command[i])
        time.sleep(t)

        self.motors.stopMotors()
        time.sleep(0.01)

    def getMotorCommands(self, current, target, verbose=0):
        """
        Uses mechanum control equations to compute motor powers for each motor
            to move along a vector between the provided current/target points
        This function does not take into account whether or not the robot has
            reached the target, and does not scale speed based on distance to
            target.

        Motor powers are ordered 1-4, given a robot with motors in the
        following position, and forward defined as:
        #    1 ---- 2     .
        #    |      |    / \
        #    |      |     |
        #    3 ---- 4     |

        # Mecanum Control:
        # https://www.roboteq.com/index.php/component/easyblog/entry/driving-mecanum-wheels-omnidirectional-robots?Itemid=1208
        # This pdf has accurate equations: http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf

        @param current DirectedPoint of robot current position
        @param target DirectedPoint of robot target position
        @param verbose Prints debugging output
        @return [V1, V2, V3, V4] list of motor powers for each robot
        """

        # Create global target directional vector
        gtarget_dpt = target - current
        # Motor directions of movement and the desired axes of movement are 
        # misaligned. Swapping x and y fixes this problem for motor command
        # computation. Theta is flipped to match global axes
        gtarget_dpt.x, gtarget_dpt.y = gtarget_dpt.y, gtarget_dpt.x
        gtarget_dpt.theta = -(math.radians(gtarget_dpt.theta) % (2 * math.pi))
        if verbose:
            print("global target dpt", gtarget_dpt)

        # Create target_dpt in local coordinates, relative to robot axis of
        # movement. This is accomplished by rotating the (x,y) vector of the
        # global axis to match that of the local one.
        # Rotate target_dpt.x,y by target_dpt.theta
        target_dpt = DirectedPoint(
            gtarget_dpt.x*math.cos(gtarget_dpt.theta) - gtarget_dpt.y*math.sin(gtarget_dpt.theta),
            gtarget_dpt.y*math.cos(gtarget_dpt.theta) + gtarget_dpt.x*math.sin(gtarget_dpt.theta),
            gtarget_dpt.theta)
        if verbose:
            print("local target dpt", target_dpt)

        ### setup mecanum control params ###
        # angle to translate at, radians 0-2pi
        target_angle = (math.atan2(target_dpt.y, target_dpt.x)) % (2 * math.pi)
        if verbose > 0:
            print("Target translation angle:", math.degrees(target_angle))
        
        # speed robot moves at, [-1, 1], original 1
        # stop if close to global goal
        magnitude = gtarget_dpt.x**2 + gtarget_dpt.y**2
        target_speed = 0.0
        if magnitude > cst.EPSILON_LARGE:
            target_speed = 0.3

        # how quickly to change robot orientation [-1, 1], original 0
        target_rot_speed = 0.0
        # Calibrated values for reasonable performance
        # 0.15 chosen for theta correction range, 0.13 is speed multiplier
        if abs(target_dpt.theta) > 0.15 and abs(target_dpt.theta) < (2 * math.pi) - 0.15:
            rot_speed_multiplier = 0
            if abs(target_dpt.theta) > math.pi:
                rot_speed_multiplier = 1
            else:
                rot_speed_multiplier = -1
            target_rot_speed = 0.13 * rot_speed_multiplier # default 0.13

    
        # Compute motors, 1-4, with forward direction as specified:
        pi4 = math.pi / 4.0
        V1 = target_speed * math.sin(target_angle + pi4) + target_rot_speed
        V2 = target_speed * math.cos(target_angle + pi4) - target_rot_speed
        V3 = target_speed * math.cos(target_angle + pi4) + target_rot_speed
        V4 = target_speed * math.sin(target_angle + pi4) - target_rot_speed
        motor_powers = [V1, V2, V3, V4]
        return self._rescaleMotorPower(motor_powers)

    def _rescaleMotorPower(self, motor_powers):
        """
        Internal function.
        Scales motor powers from the range [-1, 1] to [-255, 255].
        These motor powers can be directly used as inputs to the Motors class
        to move the motors.

        @param motor_powers List of 4 motor powers on the scale [-1, 1]
        @return motor_powers Motor powers rescaled [-255, 255]
        """
        min_scale = -1
        max_scale = 1
        for i in range(0, 4):
            motor_powers[i] = (motor_powers[i] - min_scale) / (max_scale - min_scale)
            motor_powers[i] = int((motor_powers[i] * (255 * 2)) - 255)
        return motor_powers

    def _message_watchdog(self):
        while True:
            if self._stop_thread is True:
                return
            if abs(time.time() - self.message_timer) > cst.MESSAGE_TIMEOUT:
                print("Message timer overrun, stopping motors")
                self.motors.stopMotors()
                return

    def close(self):
        """
        Stops motor movement and ends any threads.
        """
        print("Shutting down...")
        self.motors.stopMotors()
        self._stop_thread = True
        if self._watchdog_thread is not None:
            self._watchdog_thread.join()


if __name__ == "__main__":

    controller = OnboardController(robot_ip="0.0.0.0")
    controller.setup()
    controller.loop()

    # TODO create Motors() class and add test targets
    # start_pt = DirectedPoint(0, 0, 0)
    # target_pt = DirectedPoint(0, 1, 0)
    # motor_powers = controller.getMotorCommands(start_pt, target_pt)
    # print(motor_powers)

    # up = controller.getMotorCommands(start_pt, DirectedPoint(0, 1, 0))
    # left = controller.getMotorCommands(start_pt, DirectedPoint(-1, 0, 0))
    # down = controller.getMotorCommands(start_pt, DirectedPoint(0, -1, 0))
    # right = controller.getMotorCommands(start_pt, DirectedPoint(1, 0, 0))

   
    # robotcomm = RobotCommunication()
    # robotcomm.connectToOffboard()
    # while(1):
    #     msg = robotcomm.listenForMessage()
    #     if msg is None:
    #         continue
    #     else:
    #         print msg
    #         break


