"""
Onboard communication system. Handles establishing TCP connection with
main offboard controller, sending and receiving data, and processing
connections.
"""
import socket

from messages import robot_commands_pb2

from utils import constants

class RobotCommunication(object):

    def __init__(self):
        
        self.offboard_conn = None

    def connectToOffboard(self):
        """
        Onboard robot controller function.
        Establishes TCP connection with offboard controller.
        @return status Success or failure of connection attempt
        """
        address = ('0.0.0.0', constants.PORT)
        # address = (constants.OFFBOARD_IP, constants.PORT)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Establish connection
        s.bind(address)
        s.listen(1)
        conn, addr = s.accept()
        self.offboard_conn = conn

        return True

    def closeOffboardConnection(self):
        """
        Closes TCP connection with offboard controller
        @return status Success or failure of attempt to close connection
        """
        if self.offboard_conn is not None:
            self.offboard_conn.close()
            return True
        else:
            return False

    def listenForMessage(self):
        """
        Onboard robot controller function.
        Attempts to receive a command of the specified buffer size. Parses
        the command into a robot_command protobuf message and returns.
        @return data protobuf message, None if failed to read
        """
        data = robot_commands_pb2.robot_command()

        try:
            serial_data = self.offboard_conn.recv(constants.BUFFER_SIZE)
            data.ParseFromString(serial_data)
        except:
            pass

        if data.IsInitialized():
            return data
        else:
            return None