#!/usr/bin/env python

from udp import UDPhandle
import rospy
from std_msgs.msg import String


rospy.init_node("RSSI")

pub = rospy.Publisher("RSSI", String, queue_size=10)

port = 11118

start_delim = "RSSISTART\n"
end_delim = "RSSIEND\n"



@UDPhandle(port=port, start_delim=start_delim, end_delim=end_delim)
def handle_pkt(pkt=None):
    pub.publish(pkt)

handle_pkt()
