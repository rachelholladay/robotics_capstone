'''
Communication subsystem
'''
class CommunicationSystem(object):
    '''
    Contains communication subsystem
    '''
    def __init__(self):
        self.connections = [] # List of existing robot connections
        self.messages = []

    def connectToRobot(self, robot_ip):
        '''
        Establish TCP connection with robot, return connection and success status
        @param robot_ip The IP address of the robot to connect to
        @return status,connection Success status of connect attempt and connection itself
        '''
        # TODO socket stuff to connect to raspberry pi IP
        robot = None # need to actually establish connection
        self.connections.append(robot)
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


class ErrorData(object):
    """
    Contains fields for any error messages to be sent to the robots
    """
    def __init__(self):
        pass