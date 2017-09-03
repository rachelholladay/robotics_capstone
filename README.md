# Robotics Capstone Project

Team Name: Friction Force Explorers (FFE)

Group Members: Don Zheng, Neil Jassal, Yichu Jin, Rachel Holladay

# Project Description

Our team built a multi-agent robotic drawing system capable of reproducing drawings on a larger scale. Robot planning and localization is  done via offboard processing, and the robots process incoming messages to command the motors and robotic writing implements. Below is a picture of the two finished robots. A video demoing the result can be [found here.](https://youtu.be/sJ7hO-Clk_s)

![Image of Completed Robots](https://github.com/rachelholladay/robotics_capstone/blob/master/documentation/readme_documentation/final_robots.png)

# Project Implementation
For a high-level review of the project and its success, see the [validation report document.](https://github.com/rachelholladay/robotics_capstone/blob/master/documentation/readme_documentation/validation_report.pdf) This document describes each of the individual subsystems and their functions in the final system design. The below image denotes each separate subsystem, which are further described in the validation document.

![Systems Diagram](https://github.com/rachelholladay/robotics_capstone/blob/master/documentation/readme_documentation/systems_diagram.png)

## Software Design
## Controller
The main controller subsystem takes information from all offboard subsystems, and generates messages to send to the onboard robot controller to command motion. The offboard controller determines the desired state of the writing implement (engaged vs. disengaged), as well as passing the most recent localization and planning target information to the onboard system. It also handles collision detection, which pauses execution of one robot when an imminent collision is detected. Messages are sent to the onboard robot controller, which uses its known location and target location/orientation to compute a motion command for the mecanum wheel base.

## Communication
Offboard-onboard communication is done via TCP. On start, the offboard system connects wirelessly to the onboard robots. Once a stable connection is ensured, the offboard controller begins operation. The diagram below details information sent back and forth between the two controllers. Messages are built and decoded using [Protocol Buffers.](https://github.com/google/protobuf). The offboard controller runs all encoding, decoding, and sending of messages in a separate thread to reduce latency of the main controller loop.

![Communication Architecture Diagram](https://github.com/rachelholladay/robotics_capstone/blob/master/documentation/readme_documentation/sw_arch_communication.png)

## Locomotion
Robot motion is commanded by sending a series of motor commands to each of the mecanum wheel motors. Current location (includes both position and orientation) as well as goal information is passed in by the offboard system. The onboard controller then computes calibrated mecanum wheel power commands to successfully move the robot in the appropriate direction. Mecanum wheel commands are computed using the control algorithm [specified here.](http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf)

## Localization
Robot position and orientation information is calculated using [AprilTags.](https://april.eecs.umich.edu/software/apriltag.html). AprilTags are placed in all four corners of the drawing space, and on each robot.

AprilTag detection is run in C++, which runs in a separate thread from the main offboard controller. The C++ detection loop continually updates independently, which allows the offboard controller to simply poll the most recent localization data whenever it is required. AprilTag detection code can be found in `code/subsystems/apriltags/src/`. `TagDetector.h` and `TagDetector.cpp` define and implement the detection code. The file `boost_apriltags.cpp` defines the Boost module that allows the Python controller to call TagDetector functions to begin and end the detection loop.

## Writing Tool
The onboard controller changes the status of the writing tool by raising or lowering the tool itself. The status is based on incoming offboard controller communication. When enabled, it was found that small bumps in the drawing surface occasionally caused the tool the get pushed off of the surface. A simple solution of continually driving the tool into the ground proved detrimental to the motor, as it caused too much torque and burned out the writing implement motor. To remedy this, the driving motor will 'pulse' by pushing downward in short bursts, to maintain ground contact while ensuring the motor is not strained. An image of the writing implement and it's mount is below:
![Writing Implement Mount](https://github.com/rachelholladay/robotics_capstone/blob/master/documentation/readme_documentation/writing_implement_closeup.png)

## Scheduling, Distribution, and Planning (SDP)
Planning and distribution is performed as a preprocessing step to system operation. Input lines are scheduled in a greedy manner, by minimizing the total distance traveled by each robot. Collision detection is handled online, which removes the need to perform any timing-based trajectory planning. The result is, for two robots, two sets of lines that each robot will draw individually.

## Electronics & Hardware
Each robot consists of four individually powered mecanum wheels for omnidirectional motion. The writing implement also uses a single motor for raising and lowering. Below is an image of the chassis of a single robot, and the wiring design.

![Robot Chassis](https://github.com/rachelholladay/robotics_capstone/blob/master/documentation/readme_documentation/robot_chassis.png) ![Wiring Diagram](https://github.com/rachelholladay/robotics_capstone/blob/master/documentation/readme_documentation/wiring_diagram.jpg)

# Codebase Infrastructure
## File Structure

The codebase is separated into the two main controllers, and subsystems
for both onboard and offboard operation.

A majority of the code is written in Python 2.7. Localization is done in C++, and localization data passed up to the main Python controllers using Boost.

Documentation generated throughout the project design is found in folders
for various documents, labeled by purpose.

## Linting Code

We use Pylint to lint all of our python files. To run the linter with customized flags, use `pylint --rcfile='../../.pylintrc' filname.py` such that the firectory references back to the overhead folder where the rc file is located.
