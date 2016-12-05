#!/usr/bin/env python

from udp import UDPhandle
import zlib
import array
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
import sys


# need to split sensor streams onto multiple ports and send only when
# new data is available
# need to better compress point clouds

tango_clock_offset = 0

def handle_tango_clock(msg):
    global tango_clock_offset
    if tango_clock_offset == 0:
        tango_clock_offset = msg.data

clock_sub = rospy.Subscriber('/tango_clock', Float64, handle_tango_clock)
pub_point_cloud = rospy.Publisher('/point_cloud', PointCloud, queue_size=10)

rospy.init_node("pointcloud_stream")

begin_point_cloud_marker = 'POINTCLOUDSTARTINGRIGHTNOW\n' 
end_point_cloud_marker = 'POINTCLOUDENDINGRIGHTNOW\n'


@UDPhandle(port=11112, start_delim=begin_point_cloud_marker, end_delim=end_point_cloud_marker)
def handle_pkt(pkt=None):
    global pub_point_cloud	

    point_cloud_vals = array.array('f')
    point_cloud_vals.fromstring(pkt[:4*(len(pkt)//4)])
    point_cloud_vals.byteswap()
    timestamp = point_cloud_vals[0]
    point_cloud_vals = point_cloud_vals[1:]
    if len(point_cloud_vals)>1:
        point_cloud_vals = [float(p) for p in point_cloud_vals]
        msg = PointCloud()
        msg.header.stamp = rospy.Time(tango_clock_offset + float(timestamp))
        msg.header.frame_id = 'depth_camera'
        for i in range(0,len(point_cloud_vals)-3,4):
            if abs(point_cloud_vals[i]) < 10**-5 or abs(point_cloud_vals[i+1]) < 10**-5 or abs(point_cloud_vals[i+2]) < 10**-5:
                continue
            # note we are currently ignoring the confidence value
            msg.points.append(Point32(x=point_cloud_vals[i], y=point_cloud_vals[i+1], z=point_cloud_vals[i+2]))
        pub_point_cloud.publish(msg)

handle_pkt()
