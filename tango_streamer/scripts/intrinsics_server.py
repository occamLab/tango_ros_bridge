#!/usr/bin/env python 
"""
A simple echo server
"""

import socket
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud, CameraInfo
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import Float64, Float64MultiArray, Header
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
import tf

# need to split sensor streams onto multiple ports and send only when
# new data is available
# need to better compress point clouds


rospy.init_node("intrinsics_server")
camera_name = rospy.get_param('~camera_name')
port = rospy.get_param('~port_number')
pub_camera_info = rospy.Publisher('/' + camera_name + '/camera_info', CameraInfo, queue_size=10)


begin_intrinsics_marker = 'INTRINSICSSTARTINGRIGHTNOW\n' 
end_intrinsics_marker = 'INTRINSICSENDINGRIGHTNOW\n'

host = ''
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host,port))


backlog = ''


while True:
    data = sock.recv(65535)

    backlog += data

    intrinsics_remain = True

    while intrinsics_remain:
	int_start_pos = backlog.find(begin_intrinsics_marker)

	if int_start_pos == -1:
	    intrinsics_remain = False
	    break
	
	backlog = backlog[int_start_pos:]

	int_end_pos = backlog.find(end_intrinsics_marker)
	
	if int_end_pos == -1:
	    intrinsics_remain = False
	    break

	intrinsics = backlog[len(begin_intrinsics_marker):int_end_pos]
	backlog = backlog[int_end_pos+len(end_intrinsics_marker):]
	

	intrinsics_vals = intrinsics.split(",")
	print intrinsics_vals
	print "constructing message"
	ROS_timestamp = rospy.Time.now()
	msg = CameraInfo(header=Header(stamp=rospy.Time.now()),
		         width=float(intrinsics_vals[0]),
	                 height=float(intrinsics_vals[1]),
			 distortion_model='plumb_bob',
			 D=[float(n) for n in intrinsics_vals[6:]],
			 K=[float(intrinsics_vals[2]), 0, float(intrinsics_vals[4]),
 		 	 0, float(intrinsics_vals[3]), float(intrinsics_vals[5]), 0, 0, 1],
			 P =[float(intrinsics_vals[2]), 0, float(intrinsics_vals[4]), 0,
			 0, float(intrinsics_vals[3]), float(intrinsics_vals[5]), 0,
			 0, 0, 1, 0])

	pub_camera_info.publish(msg)
