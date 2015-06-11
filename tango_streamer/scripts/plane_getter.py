#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, Point, Pose
from sensor_msgs.msg import CameraInfo, PointCloud, CompressedImage
import cv2
from std_msgs.msg import Header
from tf import TransformListener
from threading import Lock

class DepthImageCreator(object):
	def __init__(self, use_depth_only):
		self.use_depth_only = use_depth_only
		self.depth_image_lock = Lock()
		self.image_list_lock = Lock()
		self.image_list = []
		self.image_list_max_size = 100
		self.downsample_factor = 2
		self.tf = TransformListener()
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
		self.clicked_point_pub = rospy.Publisher("/clicked_point",PointStamped,queue_size=10)
		self.planar_point_pub = rospy.Publisher("/second_nearest_point", PointStamped, queue_size=10)
		self.planar_point_2_pub = rospy.Publisher("/third_nearest_point", PointStamped, queue_size=10) #nts: have listener do sth like [pt1,pt2], then list[0] = list[1]; list[1] = rostopic input? Or is that not better?
		self.nearby_point_cloud_pub = rospy.Publisher("/nearby_cloud", PointCloud, queue_size=10)
		self.camera_info = None
		self.P = None
		self.depth_image = None
		self.image = None
		self.last_image_timestamp = None
		self.click_timestamp = None
		self.depth_timestamp = None
		cv2.namedWindow("depth_feed")
		cv2.namedWindow("image_feed")
		cv2.namedWindow("combined_feed")
		cv2.namedWindow("coplanar_freeze")
		cv2.setMouseCallback('image_feed',self.handle_click)
		cv2.setMouseCallback('combined_feed',self.handle_combined_click)


	def handle_click(self,event,x,y,flags,param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.click_timestamp = self.last_image_timestamp
			self.click_coords = (x*self.downsample_factor,y*self.downsample_factor)

	def process_image(self,msg):
		self.image_list_lock.acquire()
		np_arr = np.fromstring(msg.data, np.uint8)
		self.last_image_timestamp = msg.header.stamp
		self.image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
		if len(self.image_list) == self.image_list_max_size:
			self.image_list.pop(0)
		self.image_list.append((msg.header.stamp,self.image))
		self.image_list_lock.release()

	def process_camera_info(self, msg):
		self.camera_info = msg
		self.P = np.array(msg.P).reshape((3,4))
		self.K = np.array(msg.K).reshape((3,3))
		# TODO: this is necessary due to a mistake in intrinsics_server.py
		self.D = np.array([msg.D[0],msg.D[1],0,0,msg.D[2]])

	def get_nearest_image_temporally(self,timestamp):
		self.image_list_lock.acquire()
		diff_list = []
		for im_stamp,image in self.image_list:
			diff_list.append((abs((im_stamp-timestamp).to_sec()),image))
		closest_temporally = min(diff_list,key=lambda t: t[0])
		print closest_temporally[0]
		self.image_list_lock.release()
		return closest_temporally[1]

	def handle_combined_click(self,event,x,y,flags,param):
		if event == cv2.EVENT_LBUTTONDOWN:
			try:
				self.depth_image_lock.acquire()
				click_coords = (x*self.downsample_factor,y*self.downsample_factor)
				distances = []
				for i in range(self.projected_points.shape[0]):
					dist = (self.projected_points[i,0,0] - click_coords[0])**2 + (self.projected_points[i,0,1] - click_coords[1])**2
					distances.append(dist)
				by_distances = np.argsort(distances)
				three_d_coord = self.points_3d[:,by_distances[0]]
				# next_nearest_coord = self.points_3d[:, by_distances[1]]
				# third_nearest_coord = self.points_3d[:, by_distances[2]]
				# again, we have to reshuffle the coordinates due to differences in ROS Tango coordinate systems
				point_msg = PointStamped(header=Header(stamp=self.depth_image_timestamp,
													   frame_id="depth_camera"),
										 point=Point(y=three_d_coord[0],
												 	 z=three_d_coord[1],
												 	 x=three_d_coord[2]))
				# pm2 = PointStamped(header=Header(stamp=self.depth_image_timestamp,
													   # frame_id="depth_camera"),
										 # point=Point(y=next_nearest_coord[0],
												 	 # z=next_nearest_coord[1],
												 	 # x=next_nearest_coord[2]))
				# pm3 = PointStamped(header=Header(stamp=self.depth_image_timestamp,
													   # frame_id="depth_camera"),
										 # point=Point(y=third_nearest_coord[0],
												 	 # z=third_nearest_coord[1],
												 	 # x=third_nearest_coord[2]))
				nearby_cloud = PointCloud()
				for i in range(16):
					nearby_cloud.points.append(self.points_3d[:, by_distances[i]])
				self.nearby_point_cloud_pub.publish(nearby_cloud)
				
				self.tf.waitForTransform("depth_camera",
										 "odom",
										 self.depth_image_timestamp,
										 rospy.Duration(1.0))
				transformed_coord = self.tf.transformPoint('odom', point_msg)
				# tc2 = self.tf.transformPoint('odom', pm2)
				# tc3 = self.tf.transformPoint('odom', pm3)
				self.clicked_point_pub.publish(transformed_coord)
				# self.planar_point_pub.publish(tc2)
				# self.planar_point_2_pub.publish(tc3)
				self.depth_image_lock.release()
			except Exception as ex:
				print "Encountered an errror! ", ex
				self.depth_image_lock.release()


	def process_point_cloud(self, msg):
		self.depth_image_lock.acquire()
		try:
			if self.P == None:
				self.depth_image_lock.release()
				return
			self.depth_image_timestamp = msg.header.stamp
			self.depth_image = np.zeros((self.camera_info.height, self.camera_info.width, 3),dtype=np.uint8)
			self.points_3d = np.zeros((3,len(msg.points))).astype(np.float32)
			depths = []
			for i,p in enumerate(msg.points):
				# this is weird due to mismatches between Tango coordinate system and ROS
				self.points_3d[:,i] = np.array([p.y, p.z, p.x])
				depths.append(p.x)
			self.projected_points, dc = cv2.projectPoints(self.points_3d.T,
												 		  (0,0,0),
												 		  (0,0,0),
												 		  self.K,
														  self.D)

			if self.click_timestamp != None and msg.header.stamp > self.click_timestamp:
				distances = []
				for i in range(self.projected_points.shape[0]):
					dist = (self.projected_points[i,0,0] - self.click_coords[0])**2 + (self.projected_points[i,0,1] - self.click_coords[1])**2
					distances.append(dist)
				three_d_coord = self.points_3d[:,np.argmin(distances)]
				# again, we have to reshuffle the coordinates due to differences in ROS Tango coordinate systems
				point_msg = PointStamped(header=Header(stamp=msg.header.stamp,
													   frame_id="depth_camera"),
										 point=Point(y=three_d_coord[0],
													 z=three_d_coord[1],
													 x=three_d_coord[2]))
				transformed_coord = self.tf.transformPoint('odom', point_msg)
				self.clicked_point_pub.publish(transformed_coord)
				self.click_timestamp = None

			# do equalization
			depths_equalized = np.zeros((len(depths),1))
			for idx,(i,depth) in enumerate(sorted(enumerate(depths), key=lambda x: -x[1])):
				depths_equalized[i] = idx/float(len(depths)-1)
			for i in range(self.projected_points.shape[0]):
				if not(np.isnan(self.projected_points[i,0,0])) and not(np.isnan(self.projected_points[i,0,1])):
					self.depth_image[int(self.projected_points[i,0,1]),int(self.projected_points[i,0,0]),:] = int(depths_equalized[i]*255.0)
			self.depth_image_lock.release()
		except Exception as ex:
			print "Encountered an errror! ", ex
			self.depth_image_lock.release()

	def run(self):
		r = rospy.Rate(10)
		while not(rospy.is_shutdown()):
			cv2.waitKey(5)
			if self.depth_image != None:
				# dilate the depth image for display since it is so sparse
				if not(self.depth_image_lock.locked()):
					kernel = np.ones((5,5),'uint8')
					self.depth_image_lock.acquire()
					cv2.imshow("depth_feed", cv2.resize(cv2.dilate(self.depth_image, kernel),(self.depth_image.shape[1]/self.downsample_factor,
																	  					  self.depth_image.shape[0]/self.downsample_factor)))
					self.depth_image_lock.release()

			if self.image != None:
				cv2.imshow("image_feed", cv2.resize(self.image,(self.image.shape[1]/self.downsample_factor,
															self.image.shape[0]/self.downsample_factor)))
			if not(self.use_depth_only) and self.image != None and self.depth_image != None:
				kernel = np.ones((3,3),'uint8')
				self.depth_image_lock.acquire()
				nearest_image = self.get_nearest_image_temporally(self.depth_image_timestamp)
				ret, depth_threshed = cv2.threshold(self.depth_image,1,255,cv2.THRESH_BINARY)
				combined_img = (cv2.dilate(depth_threshed,kernel)*0.2 + nearest_image*0.8).astype(dtype=np.uint8)
				cv2.imshow("combined_feed", cv2.resize(combined_img,(self.image.shape[1]/self.downsample_factor,
													   self.image.shape[0]/self.downsample_factor)))

				self.depth_image_lock.release()

			if self.depth_image != None and self.use_depth_only:
				kernel = np.ones((5,5),'uint8')
				self.depth_image_lock.acquire()
				ret, depth_threshed = cv2.threshold(self.depth_image,1,255,cv2.THRESH_BINARY)
				combined_img = (cv2.dilate(depth_threshed,kernel)).astype(dtype=np.uint8)
				cv2.imshow("combined_feed", cv2.resize(combined_img,(self.image.shape[1]/self.downsample_factor,
													   self.image.shape[0]/self.downsample_factor)))

				self.depth_image_lock.release()

			r.sleep()

if __name__ == '__main__':
	rospy.init_node('make_depth_image')
	use_depth_only = rospy.get_param('/use_depth_only',False)
	node = DepthImageCreator(use_depth_only)
	node.run()