# Robotics Capstone Project

Team Name: Friction Force Explorers (FFE)
Group Members: Don Zheng, Neil Jassal, Yichu Jin, Rachel Holladay

## Project Description

Our team built a multi-agent robotic drawing system capable of reproducing drawings on a larger scale. Robot planning and localization are done via
offboard processing, and the robots process incoming messages to command
the motors and robot writing implements.

## File Structure

The codebase is separated into the two main controllers, and subsystems
for both onboard and offboard operation.

Documentation generated throughout the project design is found in folders
for various documents, labeled by purpose.

## Linting Code

We use Pylint to lint all of our python files. To run the linter with customized flags, use `pylint --rcfile='../../.pylintrc' filname.py` such that the firectory references back to the overhead folder where the rc file is located.
