'''
Communication subsystem

Requires: pip install protobuf
Requires: apt-get install protobuf-compiler
'''
class CommunicationSystem:
	'''
	Contains communication subsystem
	'''
	def __init__(self):
		self.connections = [] # List of existing robot connections

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

	def sendTCPMessage(self, locomotion=None, writing=None, error=None):
		'''
		Construct 2 proto3 messages, one for each robot. Messages concatenate
		commands from locomotion, writing, and error-reports from the offboard
		system into one proto3 message to send to each robot.

		@param locomotion Locomotion commands for each robot
		@param writing Writing implement commands for each robot
		@param error Error commands for each robot
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
