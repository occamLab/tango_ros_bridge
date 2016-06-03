#!/usr/bin/env python

import socket
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
        tango_clock_offset = msg.data

clock_sub = rospy.Subscriber('/tango_clock', Float64, handle_tango_clock)
pub_point_cloud = rospy.Publisher('/point_cloud', PointCloud, queue_size=10)

rospy.init_node("pointcloud_stream")

host = ''
port = 11112

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host,port))

begin_point_cloud_marker = 'POINTCLOUDSTARTINGRIGHTNOW\n' 
end_point_cloud_marker = 'POINTCLOUDENDINGRIGHTNOW\n'

backlog = ''

while True:

    data = sock.recv(65535)
    
    if not data:
        continue

    #add it to the to-be-processed data
    backlog += data
    
    #Mark that there might be pictures in `backlog`
    pointcloud_remain = True

    #Loop to search backlog for pictures
    while pointcloud_remain:
	#Grab pcloud, and optimize buffer on data absent
	pcloud_begin_loc = backlog.find(begin_point_cloud_marker)

	if pcloud_begin_loc == -1:
	    pointcloud_remain = False
	    break
	
	backlog = backlog[pcloud_begin_loc:]
	pcloud_end_loc = backlog.find(end_point_cloud_marker)

	if pcloud_end_loc == -1:
	    pointcloud_remain = False
	    break

        point_cloud = backlog[len(begin_point_cloud_marker):pcloud_end_loc]
 	backlog = backlog[pcloud_end_loc+len(end_point_cloud_marker):]
	
	print len(point_cloud)
        point_cloud_vals = array.array('f')
        point_cloud_vals.fromstring(point_cloud[0:-1])
        point_cloud_vals.byteswap()
        timestamp = point_cloud_vals[0]
        point_cloud_vals = point_cloud_vals[1:]
        if len(point_cloud_vals)>1:
            point_cloud_vals = [float(p) for p in point_cloud_vals]
            msg = PointCloud()
            msg.header.stamp = rospy.Time(tango_clock_offset + float(timestamp))
            msg.header.frame_id = 'depth_camera'
            for i in range(0,len(point_cloud_vals)-2,3):
                msg.points.append(Point32(y=point_cloud_vals[i], z=point_cloud_vals[i+1], x=point_cloud_vals[i+2]))
            
	    pub_point_cloud.publish(msg)
        print "timestamp", timestamp
        print "num vals ", len(point_cloud_vals)/3
        print "bytes in pc message ", len(point_cloud)

	
