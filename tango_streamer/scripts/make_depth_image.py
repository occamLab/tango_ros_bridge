#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo, PointCloud, CompressedImage
import cv2

class DepthImageCreator(object):
	def __init__(self):
		rospy.Subscriber("/camera_info",
						 CameraInfo,
						 self.process_camera_info,
						 queue_size=10)
		rospy.Subscriber("/point_cloud",
						 PointCloud,
						 self.process_point_cloud,
						 queue_size=10)
		rospy.Subscriber("/camera/image_raw/compressed",
						 CompressedImage,
						 self.process_image,
						 queue_size=10)
		self.camera_info = None
		self.P = None
		self.depth_image = None
		self.image = None
		cv2.namedWindow("depth_feed")
		cv2.namedWindow("image_feed")

	def process_image(self,msg):
		np_arr = np.fromstring(msg.data, np.uint8)
		self.image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

	def process_camera_info(self, msg):
		self.camera_info = msg
		self.P = np.array(msg.P).reshape((3,4))
		self.K = np.array(msg.K).reshape((3,3))
		# TODO: this is necessary due to a mistake in intrinsics_server.py
		self.D = np.array([msg.D[0],msg.D[1],0,0,msg.D[2]])

	def process_point_cloud(self, msg):
		if self.P == None:
			return
		self.depth_image = np.zeros((self.camera_info.height, self.camera_info.width, 3),dtype=np.uint8)
		points_3d = np.zeros((3,len(msg.points))).astype(np.float32)
		depths = []
		for i,p in enumerate(msg.points):
			# this is weird due to mismatches between Tango coordinate system and ROS
			points_3d[:,i] = np.array([p.y, p.z, p.x])
			depths.append(p.x)
		projected_points, dc = cv2.projectPoints(points_3d.T,
											 	 (0,0,0),
											 	 (0,0,0),
											 	 self.K,
											 	 self.D)

		# do equalization
		depths_equalized = np.zeros((len(depths),1))
		for idx,(i,depth) in enumerate(sorted(enumerate(depths), key=lambda x: -x[1])):
			depths_equalized[i] = idx/float(len(depths)-1)

		for i in range(projected_points.shape[0]):
			if not(np.isnan(projected_points[i,0,0])) and not(np.isnan(projected_points[i,0,1])):
				self.depth_image[int(projected_points[i,0,1]),int(projected_points[i,0,0]),:] = int(depths_equalized[i]*255.0)

		# dilate the depth image since it is so sparse
		kernel = np.ones((11,11),'uint8')
		self.depth_image = cv2.dilate(self.depth_image, kernel)

	def run(self):
		r = rospy.Rate(10)
		while not(rospy.is_shutdown()):
			cv2.waitKey(5)
			if self.depth_image != None:
				cv2.imshow("depth_feed", self.depth_image)
			if self.image != None:
				cv2.imshow("image_feed", self.image)
			r.sleep()

if __name__ == '__main__':
	rospy.init_node('make_depth_image')
	node = DepthImageCreator()
	node.run()