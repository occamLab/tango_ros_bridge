#!/usr/bin/env python

from udp import UDPhandle
import rospy
from std_msgs.msg import String, Float64
from tango_streamer.msg import Rf_stamped

rospy.init_node("RSSI")

pub = rospy.Publisher("RSSI", Rf_stamped, queue_size=10)
tango_time = rospy.Subscriber("/tango_clock", Float64, most_recent_ts)

port = 11118

start_delim = "RSSISTART\n"
end_delim = "RSSIEND\n"

ts = 0

def most_recent_ts(msg):
    global ts    
    ts = float(msg.data)


@UDPhandle(port=port, start_delim=start_delim, end_delim=end_delim)
def handle_pkt(pkt=None):
    global ts
    out = Rf_stamped(header=Header(stamp=rospy.Time.now() + ts, frame_id="device"), data=str(pkt))
    pub.publish(out)

handle_pkt()
