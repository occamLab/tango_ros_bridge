#!/usr/bin/env python

"""
Pose server for tango to ROS
"""
from udp import UDPhandle
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import Float64, Float64MultiArray, String, Int32
import sys
from tf.transformations import *
from math import pi
import numpy as np
import tf

""" Keeps track of whether we have a valid clock offset between
    ROS time and Tango time.  We don't care too much about
    how close these two are synched, we mostly just care that
    all nodes use the same offset """
tango_clock_valid = False
tango_clock_offset = -1.0

rospy.init_node("pose_server")
last_timestamp = rospy.Time.now()

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

br = tf.TransformBroadcaster()

def get_inverse_transform(quat, trans):
    A = quaternion_matrix(quat)
    A[:-1,-1] = trans
    Ainv = np.linalg.inv(A)
    pose_vals_inverse = np.hstack((Ainv[:-1,-1], quaternion_from_matrix(Ainv)));
    return pose_vals_inverse[0:3], pose_vals_inverse[3:]

def handle_pose(pose_vals):
    global br
    global tango_clock_valid
    global tango_clock_offset
    global last_timestamp
    tango_timestamp = pose_vals[-3]
    tango_status_code = pose_vals[-2]
    print tango_status_code
    # features_tracked = int(float(pose_vals[-1]))
    # if features_tracked != -1:
    #     pub_feature_track_status.publish(Int32(features_tracked))
    
    sc = int(float(tango_status_code))
    status_code_to_msg = ['TANGO_POSE_INITIALIZING','TANGO_POSE_VALID','TANGO_POSE_INVALID','TANGO_POSE_UNKNOWN']
    pub_pose_status.publish(String(status_code_to_msg[sc]))
    pose_vals = [float(p) for p in pose_vals[0:-3]]
    print "received a pose", pose_vals
    
    if status_code_to_msg[sc] != 'TANGO_POSE_VALID':
        return

    ROS_timestamp = rospy.Time.now()
    print "new offset",  ROS_timestamp.to_time() - float(tango_timestamp)

    if not(tango_clock_valid):
        tango_clock_offset = ROS_timestamp.to_time() - float(tango_timestamp)
        tango_clock_valid = True
    # publish the offset so other servers can find out about it
    pub_clock.publish(tango_clock_offset)
    
    msg = PoseStamped()
    msg.header.stamp = rospy.Time(tango_clock_offset + float(tango_timestamp))
    msg.header.frame_id = coordinate_frame

    # convert from start of service coordinate system to ROS (https://developers.google.com/tango/apis/c/reference/struct/tango-x-y-zij#xyz) to ROS coordinate system conventions
    # new x axis is the old y-axis, new y-axis is the negative x-axis
    # new roll is the old pitch, new pitch is negative of the old roll
    msg.pose.position.x = pose_vals[1]
    msg.pose.position.y = -pose_vals[0]
    msg.pose.position.z = pose_vals[2]

    q_trans = [pose_vals[4], -pose_vals[3], pose_vals[5], pose_vals[6]]

    msg.pose.orientation.x = q_trans[0]
    msg.pose.orientation.y = q_trans[1]
    msg.pose.orientation.z = q_trans[2]
    msg.pose.orientation.w = q_trans[3]

    delta_t = (msg.header.stamp - last_timestamp).to_sec()
    if delta_t < .01 or delta_t > .03:
        print delta_t, float(tango_timestamp)
    last_timestamp = msg.header.stamp
    device = "Phab2Pro"
    if device == 'Yosemite':
        # publish static transform
        br.sendTransform([-0.052506376 , -0.0077469592,  0.0075513193],
                         [  4.1631350369e-03,  -8.2141334410e-04,  -5.2960722061e-03, 9.9997697234e-01],
                         msg.header.stamp,      # use the same time stamp as the pose update
                         "fisheye_camera",
                         "depth_camera")

        br.sendTransform([ 0.0612489982, -0.0012502772,  0.0025526363],
                         [ 0.9899618129, -0.0019678283, -0.0044808391, -0.1412503409],
                         msg.header.stamp,      # use the same time stamp as the pose update
                         "depth_camera",
                         "device")
    else:
        # assume we are using the Phab2 pro
        # publish static transform
        br.sendTransform([0.050398, 0.000514, -0.000414],
                         [0.001877, 0.003729, 0.999991, 0.001068],
                         msg.header.stamp,      # use the same time stamp as the pose update
                         "fisheye_camera",
                         "color_camera")

        br.sendTransform([0.010961, 0.000060, 0.001914],
                         [-0.000778, 0.002971, 0.999995, -0.000534],
                         msg.header.stamp,      # use the same time stamp as the pose update
                         "color_camera",
                         "depth_camera")

        br.sendTransform([ 0.006841, 0.035824, -0.002991],
                         [ -2.09300111e-03,   2.37000126e-04,  -7.14237379e-01, 6.99900371e-01],
                         msg.header.stamp,      # use the same time stamp as the pose update
                         "depth_camera",
                         "device")
    # TODO: could transform directly to real_device and remove device_ros
    br.sendTransform([ 0.0, 0.0, 0.0 ],
                     [ 0.          ,  0.          , -0.7071066662,  0.7071068962],
                     msg.header.stamp,      # use the same time stamp as the pose update
                     "device",
                     "device_ros")

    # the pose currently represents the wrong axis, rotate so that it is the x-axis
    q_trans = quaternion_multiply(q_trans, quaternion_about_axis(pi/2, [0, 1, 0]))
    msg.pose.orientation.x = q_trans[0]
    msg.pose.orientation.y = q_trans[1]
    msg.pose.orientation.z = q_trans[2]
    msg.pose.orientation.w = q_trans[3]

    br.sendTransform([0.0, 0.0, 0.0],
                     quaternion_about_axis(pi/2, [0, 1, 0]),
                     msg.header.stamp,
                     "device_ros",
                     "real_device")

    br.sendTransform([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                     [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w],
                     msg.header.stamp,
                     "real_device",
                     coordinate_frame)

    pub_pose.publish(msg)


@UDPhandle(port=port, start_delim=begin_pose_marker, end_delim=end_pose_marker)
def handle_pkt(pkt=None):
    doubles_per_pose = 10

    print pkt
    pose_vals = pkt.split(",")
    for i in range(0, len(pose_vals), doubles_per_pose):
        handle_pose(pose_vals[i:i+doubles_per_pose])
    print len(pose_vals)

    
handle_pkt()
