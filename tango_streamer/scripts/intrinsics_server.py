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

pub_camera_info = rospy.Publisher('/' + camera_name + '/camera_info', CameraInfo, queue_size=10)

host = ''
port = rospy.get_param('~port_number')
backlog = 5
size = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((host,port))
s.listen(backlog)
all_data = ''
begin_intrinsics_marker = 'INTRINSICSSTARTINGRIGHTNOW\n' 
end_intrinsics_marker = 'INTRINSICSENDINGRIGHTNOW\n'
while True:
	client, address = s.accept()
	while True:
		try:
			data = client.recv(size)
			if not data:
				break
			all_data += data
			index = all_data.find(end_intrinsics_marker)
			try:
				if index != -1:
					start = all_data.find(begin_intrinsics_marker)
					intrinsics = all_data[start+len(begin_intrinsics_marker):index]
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
									 	0, float(intrinsics_vals[3]), float(intrinsics_vals[5]),
									 	0, 0, 1],
									 P =[float(intrinsics_vals[2]), 0, float(intrinsics_vals[4]), 0,
									 	0, float(intrinsics_vals[3]), float(intrinsics_vals[5]), 0,
									 	0, 0, 1, 0])

					pub_camera_info.publish(msg)
					all_data = all_data[index+len(end_intrinsics_marker):]
			except Exception as e:
				print e
				# assume we had a bogus message
				all_data = ""
		except socket.error, msg:
			sys.stderr.write('ERROR: %s\n' % msg)
			#probably got disconnected
			all_data = ''
			print "DISCONNECTED"
			break
	client.close()
