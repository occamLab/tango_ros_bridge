#!/usr/bin/env python

"""
A simple echo server
"""

import socket
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
import sys

# need to split sensor streams onto multiple ports and send only when
# new data is available
# need to better compress point clouds

pub_camera = rospy.Publisher('/camera/image_raw/compressed', CompressedImage, queue_size=10)

rospy.init_node("image_server")

host = ''
port = 11111
backlog = 5
size = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((host,port))
s.listen(backlog)
all_data = ''
begin_frame_marker = 'DEPTHFRAMESTARTINGRIGHTNOW\n' 
end_frame_marker = 'DEPTHFRAMEENDINGRIGHTNOW\n'
frame_num = 0
while True:
	client, address = s.accept()
	while True:
		try:
			data = client.recv(size)
			if not data:
				break
			all_data += data
			index = all_data.find(end_frame_marker)
			try:
				if index != -1:
					print "FOUND END FRAME"
					start = all_data.find(begin_frame_marker)
					jpg = all_data[start+len(begin_frame_marker):index]
					msg = CompressedImage()
					msg.header.stamp = rospy.Time.now()
					msg.data = jpg
					msg.format = 'jpeg'
					pub_camera.publish(msg)
					frame_num += 1
					all_data = all_data[index+len(end_frame_marker):]
					print frame_num
			except:
				# assume we had a bogus message
				all_data = ""
		except socket.error, msg:
			sys.stderr.write('ERROR: %s\n' % msg)
			#probably got disconnected
			all_data = ''
			print "DISCONNECTED"
			break
	print "CLOSING!"
	client.close()