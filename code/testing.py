"""
Code for testing and demos

Deprecated tests for incremental testing onboard the robot.
"""
import time

from onboard.motors import Motors


def writingTest2():
    m = Motors()
    m.stopMotors()

    m.write(0)
    time.sleep(5)
    m.write(1)


def writingImplementTest():
    m = Motors()
    print("lower tool")
    m.write(0)
    time.sleep(2.5)
    print("raise tool")
    m.write(1)


def demoMoveMotorsTime(motors, command, t):
    """
    Commands all motors using a given command (such as DIR_UPLEFT) for a time
    in seconds.
    @param motors Motors object
    @param command Motor command to run
    @param t Time in seconds to move for
    """
    print("Moving", command, " for", time, " seconds.")
    for i in range(0, 4):
        motors.commandMotor(i, command[i])
    time.sleep(t)

    motors.stopMotors()
    time.sleep(0.5)


# def demoOnboardSquare():
#     """
#     Commands the motors to move in a timing-based square, no rotation.
#     Uses only onboard controller and does not attempt to connect via TCP
#     """
#     # Setup constants
#     START = DirectedPoint(0, 0, 0)
#     DIR_UP = controller.getMotorCommands(start_pt, DirectedPoint(0, 1, 0))
#     DIR_LEFT = controller.getMotorCommands(start_pt, DirectedPoint(-1, 0, 0))
#     DIR_DOWN = controller.getMotorCommands(start_pt, DirectedPoint(0, -1, 0))
#     DIR_RIGHT = controller.getMotorCommands(start_pt, DirectedPoint(1, 0, 0))

#     DIR_UPLEFT = controller.getMotorCommands(start_pt, DirectedPoint(-1, 1, 0))
#     DIR_UPRIGHT = controller.getMotorCommands(start_pt, DirectedPoint(1, 1, 0))
#     DIR_DOWNLEFT = controller.getMotorCommands(start_pt,
#        DirectedPoint(-1, -1, 0))
#     DIR_DOWNRIGHT = controller.getMotorCommands(start_pt,
#        DirectedPoint(1, -1, 0))

#     m = Motors()
#     print("up", DIR_UP)
#     for i in range(0,4):
#         power = DIR_UP[i]
#         m.commandMotor(i, power)
#     time.sleep(2)

#     m.stopMotors()
#     time.sleep(1)

#     print("left", DIR_LEFT)
#     for i in range(0,4):
#         m.commandMotor(i, DIR_LEFT[i])
#     time.sleep(2)

#     m.stopMotors()
#     time.sleep(1)

#     print("down", DIR_DOWN)
#     for i in range(0,4):
#         m.commandMotor(i, DIR_DOWN[i])
#     time.sleep(2)

#     m.stopMotors()
#     time.sleep(1)

#     print("right", DIR_RIGHT)
#     for i in range(0,4):
#         m.commandMotor(i, DIR_RIGHT[i])
#     time.sleep(2)

#     m.stopMotors()
#     time.sleep(1)

#     time.sleep(3)
#     m.stopMotors()


if __name__ == "__main__":
    # Run current testing code
    writingTest2()
