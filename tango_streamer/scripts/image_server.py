#!/usr/bin/env python

import socket
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

#socket details
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host, port))

#data storage
backlog = ""



#Loop to sit on socket and grab data
while True:

    #blocks until can grab udp packet
    data = sock.recv(65535)
    if not data:
        break

    #add it to the to-be-processed data
    backlog += data
    
    #Mark that there might be pictures in `backlog`
    pictures_remain = True

    #Loop to search backlog for pictures
    while pictures_remain: 
        jpeg_begin_loc = backlog.find(begin_frame_marker)
        jpeg_end_loc = backlog.find(end_frame_marker)
        
        #It found a picture!
        if jpeg_begin_loc != -1 and  jpeg_end_loc != -1:
            jpg = backlog[jpeg_begin_loc+len(begin_frame_marker):jpeg_end_loc]
            backlog = backlog[jpeg_eng_loc+len(end_frame_marker):]

            ts_begin_loc = jpg.find(begin_timestamp_marker)
            ts_end_loc = jpg.find(end_timestamp_marker)

            if ts_begin_loc == -1 or ts_end_loc == -1:
                #something's fishy, discard jpeg
                print("JPEG discarded, malformed data")
                break
                
            ts = jpg[ts_begin_loc+len(begin_timestamp_marker):ts_end_marker]
            jpg = jpg[ts_end_loc+len(end_timestamp_marker):]

            print "{} bytes of JPEG recvd".format(len(jpg))
            msg = CompressedImage()
            msg.header.stamp  = rospy.Time(tango_clock_offset + float(ts))
            msg.header.frame_id = camera_name
            msg.data = jpg
            msg.format = 'jpeg'
            pub_camera.publish(msg)


        #no end bytes in sight
        elif jpeg_end_loc == -1:
            pictures_remain = False

        #no beginning bytes in sight
        elif jpeg_begin_loc == -1:
            backlog = ""
            pictures_remain = False


