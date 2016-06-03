#!/usr/bin/env python

"""
Pose server for tango to ROS
"""

import socket
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import Float64, Float64MultiArray, String, Int32
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
import tf

""" Keeps track of whether we have a valid clock offset between
    ROS time and Tango time.  We don't care too much about
    how close these two are synched, we mostly just care that
    all nodes use the same offset """
tango_clock_valid = False
tango_clock_offset = -1.0

rospy.init_node("pose_server")

host = ''
port = rospy.get_param('~port_number')
pose_topic = rospy.get_param('~pose_topic')
coordinate_frame = rospy.get_param('~coordinate_frame')
pub_feature_track_status = rospy.Publisher('/tango_feature_tracking_status', Int32, queue_size=10)
pub_pose_status = rospy.Publisher('/tango_pose_status', String, queue_size=10)
pub_pose = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
pub_angles = rospy.Publisher('/tango_angles', Float64MultiArray, queue_size=10)
pub_clock = None
pub_clock = rospy.Publisher('/tango_clock', Float64, queue_size=10)

begin_pose_marker = 'POSESTARTINGRIGHTNOW\n' 
end_pose_marker = 'POSEENDINGRIGHTNOW\n'

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host, port))
backlog = ""



br = tf.TransformBroadcaster()
while True:
    data = sock.recv(4096)

    backlog += data

    pose_marker_present = True

    while pose_marker_present:
	pose_begin_loc = backlog.find(begin_pose_marker)

	if pose_begin_loc == -1:
	    pose_marker_present = False
	    break

	backlog = backlog[pose_begin_loc:]

	pose_end_loc = backlog.find(end_pose_marker)
	
	if pose_end_loc == -1:
	    pose_marker_present = False
	    break

	pose = backlog[pose_begin_loc+len(begin_pose_marker):pose_end_loc]
	backlog = backlog[pose_end_loc+len(end_pose_marker):]

	pose_vals = pose.split(",")
	tango_timestamp = pose_vals[-3]
	tango_status_code = pose_vals[-2]
	features_tracked = int(float(pose_vals[-1]))
	print features_tracked
        if features_tracked != -1:
            pub_feature_track_status.publish(Int32(features_tracked))

	sc = int(float(tango_status_code))
        status_code_to_msg = ['TANGO_POSE_INITIALIZING','TANGO_POSE_VALID','TANGO_POSE_INVALID','TANGO_POSE_UNKNOWN']
        pub_pose_status.publish(String(status_code_to_msg[sc]))
        print "published successfully!"
        pose_vals = pose_vals[0:-3]

        ROS_timestamp = rospy.Time.now()
        if not(tango_clock_valid):
            tango_clock_offset = ROS_timestamp.to_time() - float(tango_timestamp)
            tango_clock_valid = True
        # publish the offset so other servers can find out about it
        pub_clock.publish(tango_clock_offset)

        msg = PoseStamped()
        # might need to revisit time stamps
        msg.header.stamp = rospy.Time(tango_clock_offset + float(tango_timestamp))
        msg.header.frame_id = coordinate_frame

        msg.pose.position.x = float(pose_vals[0])
        msg.pose.position.y = float(pose_vals[1])
        msg.pose.position.z = float(pose_vals[2])

        # two of the rotation axes seem to be off...
        # we are fixing this in a hacky way right now
        euler_angles = euler_from_quaternion(pose_vals[3:])
        pose_vals[3:] = quaternion_from_euler(euler_angles[1],
                                              euler_angles[0]+pi/2, # this is right
                                              euler_angles[2]-pi/2)
        euler_angles_transformed = euler_from_quaternion(pose_vals[3:])
        msg2 = Float64MultiArray(data=euler_angles_transformed)
        pub_angles.publish(msg2)
    
        msg.pose.orientation.x = float(pose_vals[3])
        msg.pose.orientation.y = float(pose_vals[4])
        msg.pose.orientation.z = float(pose_vals[5])
        msg.pose.orientation.w = float(pose_vals[6])
    
        euler_angles_depth_camera = (euler_angles_transformed[0],
                                     euler_angles_transformed[1],
                                     euler_angles_transformed[2])
        pub_pose.publish(msg)
        br.sendTransform((msg.pose.position.x,
                          msg.pose.position.y,
                          msg.pose.position.z),
                         quaternion_from_euler(euler_angles_depth_camera[0],
                                               euler_angles_depth_camera[1],
                                               euler_angles_depth_camera[2]),
                         rospy.Time(tango_clock_offset + float(tango_timestamp)),
                         "device",          # this should be something different like "device"
                         coordinate_frame)

