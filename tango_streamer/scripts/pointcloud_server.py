#!/usr/bin/env python

"""
A simple echo server
"""

import socket
import rospy
import time
from sensor_msgs.msg import CompressedImage, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
import sys

# need to split sensor streams onto multiple ports and send only when
# new data is available
# need to better compress point clouds

pub_point_cloud = rospy.Publisher('/point_cloud', PointCloud, queue_size=10)

rospy.init_node("pointcloud_stream")

host = ''
port = 11112
backlog = 5
size = 100024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((host,port))
s.listen(backlog)
all_data = ''
begin_point_cloud_marker = 'POINTCLOUDSTARTINGRIGHTNOW\n' 
end_point_cloud_marker = 'POINTCLOUDENDINGRIGHTNOW\n'
frame_num = 0
while True:
	client, address = s.accept()
	while True:
		try:
			data = client.recv(size)
			if not data:
				break
			all_data += data
			index = all_data.find(end_point_cloud_marker)
			try:
				if index != -1:
					start = all_data.find(begin_point_cloud_marker)
					t = time.time()
					point_cloud = all_data[start+len(begin_point_cloud_marker):index]
					point_cloud_vals = point_cloud.split(',')
					print len(point_cloud_vals)
					if len(point_cloud_vals)>1:
						point_cloud_vals = [float(p) for p in point_cloud_vals]
						msg = PointCloud()
						msg.header.stamp = rospy.Time.now()
						msg.header.frame_id = 'depth_camera'

						for i in range(0,len(point_cloud_vals),3):
							# msg.points.append(Point32(x=point_cloud_vals[i],
							# 						  y=point_cloud_vals[i+1],
							# 						  z=point_cloud_vals[i+2]))
							msg.points.append(Point32(y=point_cloud_vals[i],
					    							  z=point_cloud_vals[i+1],
					    							  x=point_cloud_vals[i+2]))
						pub_point_cloud.publish(msg)
					all_data = all_data[index+len(end_point_cloud_marker):]
					print "num vals ", len(point_cloud_vals)/3
					print "bytes in pc message ", len(point_cloud)
					print "time to create point cloud message", (time.time() - t)
			except:
				# assume we had a bogus message
				all_data = ""
		except socket.error, msg:
			sys.stderr.write('ERROR: %s\n' % msg)
			#probably got disconnected
			all_data = ''
			break
	client.close()