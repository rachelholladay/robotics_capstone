'''
Communication subsystem

Requires: pip install protobuf
Requires: apt-get install protobuf-compiler
'''

from utils import dataStorage as storage
from messages import robot_commands_pb2

class CommunicationSystem(object):
    '''
    Contains communication subsystem.

    Message design: This class keeps track of the most recently unsent protobuf
    message, filling it with data as various other subsystems send updates. On
    sending, it sends the multiple protobuf messages to each of its existing
    connections, or robots.

    For example, in a 2 robot system, a CommunicationSystem will hold and fill
    2 unique protobuf messages. When the message is sent, it will flush the 
    protobuf message and begin again. It then waits for updates from other 
    subsystems with new and updated information for the messages.
    '''

    def __init__(self):
        self.connections = [] # List of existing robot connections
        self.messages = []

    def connectToRobot(self, robot_ip, robot_id):
        '''
        Establish TCP connection with robot, return connection and success status
        @param robot_id Integer ID to delineate individual robots
        @param robot_ip The IP address of the robot to connect to
        @return status,connection Success status of connect attempt and connection itself
        '''
        # TODO socket stuff to connect to raspberry pi IP
        robot_connection = None


        msg = robot_commands_pb2.robot_command()
        msg.robot_id = robot_id

        self.connections.append(robot_connection)
        self.messages.append(msg)
        pass


    def getTCPMessages(self):
        '''
        Request message from each robot from the list of established connections
        @return messages List of proto3 message objects from each robot
        '''
        messages = []
        for robot in self.connections:
            robot_msg = None # TODO request and receive message from robot
            messages.append(robot_msg)

        return messages

    def sendTCPMessage(self):
        '''
        Sends both proto3 messages to the respective robots via TCP connection
        @return status Success or failure status of sending messages
        '''
        status = False
        return False

    def closeTCPConnections(self):
        '''
        Closes any existing TCP messages
        @return status Success or failure of ending communications
        '''
        for robot in self.connections:
            # TODO close TCP connection
            self.connections.remove(robot)

    def generateMessage(robot, locomotion, error):
        """
        Builds the message for the specified robot consisting of locomotion,
        writing, and error data. Message is a proto3 message to be sent to
        the onboard robot controllers. These messages concatenate locomotion
        commands, which includes wheels and writing tool data, and error
        information.

        @param robot Index to specify which robot the message is for
        @param locomotion LocomotionData struct for wheels and writing tool
        @param error ErrorData struct
        """
        self.messages[robot] = None
        pass


