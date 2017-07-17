"""
Onboard communication system. Handles establishing TCP connection with
main offboard controller, sending and receiving data, and processing
connections.
"""
import socket

from messages import robot_commands_pb2
from utils import constants


class RobotCommunication(object):
    """
    This class handles onboard TCP communication with the main offboard
    controller. This handles connecting to the offboard controller, listening
    and returning messages to the main onboard controller, and closing the
    connection after robot operation.

    Recieved messages are expected to be robot_commands protobuf messages.
    
    Sample operation of the onboard robot communication class is as follows:
        comm = RobotCommunication()
        comm.connectToOffboard()
        while drawing_incomplete:
            message = comm.listenForMessage()
            parse received message
    """
    def __init__(self):
        """
        Creates the onboard communication internal object
        """
        self.offboard_conn = None

    def connectToOffboard(self):
        """
        Onboard robot controller function.
        Establishes TCP connection with offboard controller.

        @return status Success or failure of connection attempt
        """
        address = ('0.0.0.0', constants.PORT)
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

        # Attempt to parse received protobuf message
        try:
            serial_data = self.offboard_conn.recv(constants.BUFFER_SIZE)
            data.ParseFromString(serial_data)
        except RuntimeError:
            pass

        if data.IsInitialized():
            return data
        else:
            return None
