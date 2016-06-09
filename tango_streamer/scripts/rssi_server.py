#!/usr/bin/env python

from udp import UDPhandle
import rospy
from std_msgs.msg import String, Float64


rospy.init_node("RSSI")

pub = rospy.Publisher("RSSI", String, queue_size=10)
tango_time = rospy.Subscriber("/tango_clock", Float64, most_recent_ts)

port = 11118

start_delim = "RSSISTART\n"
end_delim = "RSSIEND\n"

ts = 0

def most_recent_ts(msg):
    ts = float(msg.data)


@UDPhandle(port=port, start_delim=start_delim, end_delim=end_delim)
def handle_pkt(pkt=None):
    global ts
   
    pub.publish(str(ts) + '\n' + pkt)

handle_pkt()
