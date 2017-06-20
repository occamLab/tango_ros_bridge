#!/usr/bin/env python

from udp import UDPhandle
import zlib
import array
import rospy
import time
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import CompressedImage, PointCloud, PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud_xyz32
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
pub_point_cloud = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)

rospy.init_node("pointcloud_stream")

begin_point_cloud_marker = 'POINTCLOUDSTARTINGRIGHTNOW\n'
end_point_cloud_marker = 'POINTCLOUDENDINGRIGHTNOW\n'


@UDPhandle(port=11112, start_delim=begin_point_cloud_marker, end_delim=end_point_cloud_marker)
def handle_pkt(pkt=None):
    global pub_point_cloud
    if len(pkt) % 4 != 0:
        print "WARNING! BAD POINTCLOUD PACKET"
        return
    msg = PointCloud2()
    point_cloud_vals = array.array('B')
    point_cloud_vals.fromstring(pkt[:4*(len(pkt)//4)])
    #point_cloud_vals.byteswap()
    #timestamp = #point_cloud_vals[0:4]
    point_cloud_vals = point_cloud_vals[4:]
    if len(point_cloud_vals)>1:

        msg.header = Header(stamp = rospy.Time.now(), frame_id='depth_camera')
        msg.data = point_cloud_vals
        msg.height = 1
        msg.width = len(point_cloud_vals)/16
        msg.point_step = 16
        msg.is_dense = True
        pointfields = []
        names = ('x','y','z','c')
        for i in range(4):
            pointfields.append(PointField(name = names[i], offset = 4*i, datatype= 7, count = 0))
        msg.fields = pointfields
        #msg = create_cloud_xyz32(newheader, point_cloud_vals)
        #for i in range(0,len(point_cloud_vals)-3,4):
        #    if abs(point_cloud_vals[i]) < 10**-5 or abs(point_cloud_vals[i+1]) < 10**-5 or abs(point_cloud_vals[i+2]) < 10**-5:
        #        continue
            # note we are currently ignoring the confidence value
        #    msg.points.append(Point32(x=point_cloud_vals[i], y=point_cloud_vals[i+1], z=point_cloud_vals[i+2]))
        print(msg.width)
        pub_point_cloud.publish(msg)

handle_pkt()
