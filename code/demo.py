"""
Code for testing and demos
"""

from onboard_controller import OnboardController
from onboard.motors import Motors
from utils.geometry import DirectedPoint

START = DirectedPoint(0, 0, 0)
DIR_UP = controller.getMotorCommands(start_pt, DirectedPoint(0, 1, 0))
DIR_LEFT = controller.getMotorCommands(start_pt, DirectedPoint(-1, 0, 0))
DIR_DOWN = controller.getMotorCommands(start_pt, DirectedPoint(0, -1, 0))
DIR_RIGHT = controller.getMotorCommands(start_pt, DirectedPoint(1, 0, 0))

DIR_UPLEFT = controller.getMotorCommands(start_pt, DirectedPoint(-1, 1, 0))
DIR_UPRIGHT = controller.getMotorCommands(start_pt, DirectedPoint(1, 1, 0))
DIR_DOWNLEFT = controller.getMotorCommands(start_pt, DirectedPoint(-1, -1, 0))
DIR_DOWNRIGHT = controller.getMotorCommands(start_pt, DirectedPoint(1, -1, 0))
