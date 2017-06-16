#!/usr/bin/env python

import socket
import time
import rospy
from std_msgs.msg import Int32, String

class CommandRelay(object):
    def __init__(self):
        TCP_IP = '0.0.0.0'
        TCP_PORT = 12000

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((TCP_IP, TCP_PORT))
        self.s.listen(1)
        rospy.init_node('command_relay')
        rospy.Subscriber('/vibrate', Int32, self.process_vibration)
        rospy.Subscriber('/speech', String, self.process_speech)


    def process_vibration(self, msg):
        print msg
        try:
            self.conn.sendall("vibrate: " + str(msg.data) + "\r\n")
        except Exception as ex:
            print ex


    def process_speech(self, msg):
        print "speaking", msg
        try:
            self.conn.sendall("speech: " + msg.data + "\r\n")
        except Exception as ex:
            print ex


    def run(self):
        while not rospy.is_shutdown():
            try:
                self.conn, addr = self.s.accept()
                self.conn.setsockopt( socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                print 'Connection address:', addr
            except Exception as ex:
                self.conn.close()
                self.conn = None

if __name__ == '__main__':
    node = CommandRelay()
    node.run()