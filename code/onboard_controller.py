'''
Main onboard controller.
'''
import sys
from subsystems.communication import CommunicationSystem


class OnboardController(object):
    def __init__(self, robot_ip):
        '''
        Instantiates main subsystems based on input parameters
        '''
        self.robot_ip = robot_ip




    def _test(self):
        pass

if __name__ == "__main__":
    offboard_ip = '111.111.1.1'

    from messages import robot_commands_pb2
    import socket
    address = ('localhost', 5555)
    buf_size = 1024

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    s.bind(address)
    s.listen(1)
    serialized = None
    
    conn, addr = s.accept()
    print 'connection address: ', addr
    while 1:
        data = robot_commands_pb2.robot_command()
        serialized = conn.recv(buf_size)
        if serialized is None:
            continue

        print 'recieved: '
        data.ParseFromString(serialized)
        print data
        break

    conn.send(serialized)
    conn.close()

