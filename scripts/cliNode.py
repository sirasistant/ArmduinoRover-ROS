#!/usr/bin/env python
import rospy
import socket
from std_msgs.msg import String
from ArmduinoRover.srv import *

TCP_IP = '127.0.0.1'
TCP_PORT = 6789
BUFFER_SIZE = 1024
cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
cli.connect((TCP_IP, TCP_PORT))

def handle_request(req):
    print "Receiving %s "%(req.str)
    cli.send(req.str)
    data = cli.recv(BUFFER_SIZE)
    return cliCommResponse(data)

def cli_service():
    rospy.init_node('cli_node')
    s = rospy.Service('cli_communication', cliComm, handle_request)
    print "Ready to communicate."
    rospy.spin()

if __name__ == '__main__':
    try:
        cli_service()
    except rospy.ROSInterruptException:
        pass
    cli.close()