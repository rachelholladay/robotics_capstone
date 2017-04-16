'''
Communication subsystem

'''
import socket
import time

from messages import robot_commands_pb2

from utils import dataStorage as storage
from utils import constants

class CommunicationSystem(object):
    '''
    Contains communication subsystem. Allows for TCP communication using
    protbuf to and from the offboard and onboard systems.

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


    def connectToRobot(self, robot_id, robot_ip):
        '''
        For offboard controller.
        Attempt to establish TCP connection with robot at specified
        IP address.

        @param robot_id Integer ID to delineate individual robots
        @param robot_ip The IP address of the robot to connect to
        @return status Success status of connect attempt
        '''
        # Setup TCP socket
        address = (robot_ip, constants.PORT)
        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Attempt to connect
        try:
            conn.connect(address)
        except socket.error:
            print "Unable to connect Robot ID: ",robot_id," at IP ",robot_ip
            return False

        msg = robot_commands_pb2.robot_command()
        msg.robot_id = robot_id

        self.connections.append(conn)
        self.messages.append(msg)
        
        return True


    def getTCPMessages(self):
        '''
        For offboard controller.
        Request message from each robot from the list of established connections
        @return messages List of proto3 message objects from each robot
        '''
        messages = []
        for robot in self.connections:
            robot_msg = None # TODO request and receive message from robot
            messages.append(robot_msg)

        return messages


    def sendTCPMessages(self):
        '''
        For offboard controller.
        Sends both protobuf messages to the respective robots via TCP connection
        @return status Int status of sending messages to individual robots. 
            0: Successful transmission between all robots
            n: Number of robots for which message failed to send
        '''
        status = 0
        for i in range(len(self.connections)):
            try:
                conn = self.connections[i]
                conn.send(self.messages[i].SerializeToString())
            except socket.error:
                status += 1
        return status


    def closeTCPConnections(self):
        '''
        Closes any existing TCP messages
        @return status Success or failure of ending communications
        '''
        for i in range(len(self.connections)):
            # TODO close TCP connection
            robot_connection = self.connections[i]
            robot_connection.close()

            self.connections.remove(robot_connection)
            self.messages.remove(self.messages[i])


    def clearMessage(self, robot_id):
        """
        Clears the protobuf message for the corresponding robot id
        @param robot_id The index/ID of the proto message to clear
        """
        self.messages[robot_id].Clear()


    def generateMessage(self, robot_id, locomotion, error):
        """
        Builds the message for the specified robot consisting of locomotion,
        writing, and error data. Message is a proto3 message to be sent to
        the onboard robot controllers. These messages concatenate locomotion
        commands, which includes wheels and writing tool data, and error
        information.

        @param robot_id Index to specify which robot the message is for
        @param locomotion LocomotionData struct for wheels and writing tool
        @param error ErrorData struct
        """
        try:
            self.messages[robot_id].Clear()
            self.messages[robot_id].robot_id = robot_id

            self.messages[robot_id].robot_x = locomotion.tf_robot.x
            self.messages[robot_id].robot_y = locomotion.tf_robot.y
            self.messages[robot_id].robot_th = locomotion.tf_robot.theta

            self.messages[robot_id].target_x = locomotion.tf_target.x
            self.messages[robot_id].target_y = locomotion.tf_target.y
            self.messages[robot_id].target_th = locomotion.tf_target.theta
            
            self.messages[robot_id].stop_status = locomotion.stop_status
        except:
            pass
        


