#!/usr/bin/env python

from udp import UDPhandle
import rospy
import sys

from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage, PointCloud
from geometry_msgs.msg import PoseStamped, Point32

tango_clock_offset = 0

def handle_tango_clock(msg):
    global tango_clock_offset
    tango_clock_offset = msg.data


#rospy to interface with ROS setup
rospy.init_node("image_server")

host = '' 
port = rospy.get_param('~port_number')
camera_name = rospy.get_param('~camera_name')

clock_sub = rospy.Subscriber('/tango_clock', Float64, handle_tango_clock)
pub_camera = rospy.Publisher('/' + camera_name + '/image_raw/compressed', CompressedImage, queue_size=10)

#encoding details
begin_frame_marker = 'DEPTHFRAMESTARTINGRIGHTNOW\n'
end_frame_marker = 'DEPTHFRAMEENDINGRIGHTNOW\n'
begin_timestamp_marker = 'DEPTHTIMESTAMPSTARTINGRIGHTNOW\n'
end_timestamp_marker = 'DEPTHTIMESTAMPENDINGRIGHTNOW\n'


@UDPhandle(port=port, start_delim=begin_frame_marker, end_delim=end_frame_marker)
def handle_pkt(pkt=None):
    ts_begin_loc = pkt.find(begin_timestamp_marker)
    ts_end_loc = pkt.find(end_timestamp_marker)

    if ts_begin_loc == -1 or ts_end_loc == -1:
        #something's fishy, discard jpeg
        print "JPEG discarded, malformed data"
        return 

    try:
        ts = float(pkt[ts_begin_loc+len(begin_timestamp_marker):ts_end_loc])
    except Exception as inst:
        # occasionally we are getting packets that seem to have multiple frames in them...  This results in
        # the timestamp being malformed which causes an exception (TODO: look into what could be causing
        # this on the Android side)
        print inst
        return

    pkt = pkt[ts_end_loc+len(end_timestamp_marker):]

    print "{} bytes of JPEG recvd".format(len(pkt))
    msg = CompressedImage()
    msg.header.stamp  = rospy.Time(tango_clock_offset + float(ts))
    msg.header.frame_id = camera_name
    msg.data = pkt
    msg.format = 'jpeg'
    pub_camera.publish(msg)

handle_pkt()
