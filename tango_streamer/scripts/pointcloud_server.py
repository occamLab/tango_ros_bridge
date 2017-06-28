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
    point_cloud_vals = array.array('f')
    point_cloud_vals.fromstring(pkt[:4*(len(pkt)//4)])
    point_cloud_vals.byteswap()
    timestamp = point_cloud_vals[0]
    point_cloud_vals = point_cloud_vals[1:]

    if len(point_cloud_vals)>1:
        pointfields = []
        names = ('x','y','z')#,'c') If this is added back in, find a way to remove it for transformations.

        for i, name in enumerate(names):
            pointfields.append(PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1))
        # fix this timestamp
        msg = PointCloud2(header=Header(stamp=rospy.Time(tango_clock_offset + float(timestamp)),
                                        frame_id='depth_camera'),
                          height=1,
                          width=len(point_cloud_vals)/4,
                          point_step=16,
                          is_dense=True,
                          data=point_cloud_vals.tostring(),
                          fields=pointfields)
        pub_point_cloud.publish(msg)

handle_pkt()
