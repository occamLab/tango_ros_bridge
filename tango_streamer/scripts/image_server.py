#!/usr/bin/env python

"""
A simple echo server
"""

import socket
import rospy
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
	print "Got new offset", tango_clock_offset

rospy.init_node("image_server") 

host = ''
port = rospy.get_param('~port_number')
camera_name = rospy.get_param('~camera_name')

clock_sub = rospy.Subscriber('/tango_clock', Float64, handle_tango_clock)
pub_camera = rospy.Publisher('/' + camera_name + '/image_raw/compressed', CompressedImage, queue_size=10)

backlog = 5
size = 10**6
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((host,port))
s.listen(backlog)
all_data = ''
begin_frame_marker = 'DEPTHFRAMESTARTINGRIGHTNOW\n' 
end_frame_marker = 'DEPTHFRAMEENDINGRIGHTNOW\n'

begin_timestamp_marker = 'DEPTHTIMESTAMPSTARTINGRIGHTNOW\n' 
end_timestamp_marker = 'DEPTHTIMESTAMPENDINGRIGHTNOW\n'

timestamp = 0

frame_num = 0
while True:
	client, address = s.accept()
	while True:
		try:
			data = client.recv(size)
			if not data:
				break
			all_data += data
			print len(all_data)

			index = all_data.find(end_frame_marker)
			try:
				if index != -1:
					print "FOUND END FRAME"
					start = all_data.find(begin_frame_marker)
					jpg = all_data[start+len(begin_frame_marker):index]

					index_ts = jpg.find(end_timestamp_marker)
					try:
						if index != -1:
							start_ts = jpg.find(begin_timestamp_marker)
							timestamp = jpg[start_ts+len(begin_timestamp_marker):index_ts]
							print float(timestamp)
							jpg = jpg[index_ts+len(end_timestamp_marker):]
					except:
						# assume we had a bogus message
						all_data = ""

					print len(jpg)
					msg = CompressedImage()
					msg.header.stamp = rospy.Time(tango_clock_offset + float(timestamp))
					msg.header.frame_id = 'camera'
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