#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, Point, Pose
from sensor_msgs.msg import CameraInfo, PointCloud2, CompressedImage, Image
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from tf import TransformListener
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from threading import Lock
from math import pi
from tf.transformations import euler_matrix

class DepthImageCreator(object):
    def __init__(self):
        self.cloud_count = 0
        self.bridge = CvBridge()
        self.depth_image_lock = Lock()
        self.image_list_lock = Lock()
        self.image_list = []
        self.image_list_max_size = 100
        self.downsample_factor = 2
        self.tf = TransformListener()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/color_camera/camera_info",
                         CameraInfo,
                         self.process_camera_info,
                         queue_size=10)
        rospy.Subscriber("/point_cloud",
                         PointCloud2,
                         self.process_point_cloud,
                         queue_size=10)
        rospy.Subscriber("/color_camera/image_raw",
                         Image,
                         self.process_image,
                         queue_size=10)
        self.clicked_point_pub = rospy.Publisher("/clicked_point",PointStamped,queue_size=10)
        self.camera_info = None
        self.depth_image = None
        self.image = None
        self.depth_timestamp = None
        cv2.namedWindow("combined_feed")
        cv2.setMouseCallback('combined_feed',self.handle_combined_click)

    def process_image(self,msg):
        self.image_list_lock.acquire()
        self.image = self.bridge.imgmsg_to_cv2(msg)

        if len(self.image_list) == self.image_list_max_size:
            self.image_list.pop(0)
        self.image_list.append((msg.header.stamp,self.image))
        self.image_list_lock.release()

    def process_camera_info(self, msg):
        if self.camera_info != None:
            return
        self.camera_info = msg
        self.K = np.array(msg.K).reshape((3,3))
        # TODO: this is necessary due to a mistake in intrinsics_server.py
        self.D = np.array([msg.D[0],msg.D[1],0,0,msg.D[2]])

    def get_nearest_image_temporally(self,timestamp):
        self.image_list_lock.acquire()
        diff_list = []
        for im_stamp,image in self.image_list:
            diff_list.append((abs((im_stamp-timestamp).to_sec()),image, im_stamp))
        closest_temporally = min(diff_list,key=lambda t: t[0])
        print closest_temporally[0]
        self.image_list_lock.release()
        return closest_temporally[1], closest_temporally[2]

    def handle_combined_click(self,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            try:
                self.depth_image_lock.acquire()
                click_coords = (x*self.downsample_factor, y*self.downsample_factor)

                distances = [(p[0,0] - click_coords[0])**2 + (p[0,1] - click_coords[1])**2  for p in self.projected_points]
                three_d_coord = self.points_3d[np.argmin(distances),:]

                # again, we have to reshuffle the coordinates due to differences in ROS Tango coordinate systems
                point_msg = PointStamped(header=Header(stamp=self.depth_image_timestamp,
                                                       frame_id="color_camera"),
                                         point=Point(x=three_d_coord[0],
                                                     y=three_d_coord[1],
                                                     z=three_d_coord[2]))
                self.tf.waitForTransform("color_camera",
                                         "odom",
                                         self.depth_image_timestamp,
                                         rospy.Duration(1.0))
                transformed_coord = self.tf.transformPoint('odom', point_msg)
                self.clicked_point_pub.publish(transformed_coord)
                self.depth_image_lock.release()

            except Exception as ex:
                print "Encountered an errror! ", ex
                self.depth_image_lock.release()


    def process_point_cloud(self, msg):
        self.cloud_count += 1
        if self.cloud_count % 4 != 0:
            return
        if self.K == None or self.D == None:
            return
        self.depth_image_lock.acquire()
        try:
            self.depth_image_timestamp = msg.header.stamp
            _, nearest_image_timestamp = self.get_nearest_image_temporally(self.depth_image_timestamp)
            try:
                transform = self.tf_buffer.lookup_transform_full(
                                         target_frame='depth_camera',
                                         target_time=msg.header.stamp,
                                         source_frame='color_camera',
                                         source_time=nearest_image_timestamp,
                                         fixed_frame='odom',
                                         timeout=rospy.Duration(1.0)
                                )
            except Exception as inst:
                print "transform error", inst

            msg = do_transform_cloud(msg, transform)

            self.depth_image = np.zeros((self.camera_info.height, self.camera_info.width, 3),dtype=np.uint8)
            points = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z', 'c'))
            depths = [p[2] for p in points]
            points = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z', 'c'))

            self.points_3d = np.asarray([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
            #self.points_3d = self.points_3d.dot(euler_matrix(0, 0, pi)[0:3,0:3])

            self.projected_points, dc = cv2.projectPoints(self.points_3d,
                                                          (0,0,0),
                                                          (0,0,0),
                                                          self.K,
                                                          self.D)

            for i in range(self.projected_points.shape[0]):
                if 0 <= int(self.projected_points[i,0,1]) < self.depth_image.shape[0] and \
                   0 <= int(self.projected_points[i,0,0]) < self.depth_image.shape[1] and \
                   not(np.isnan(self.projected_points[i,0,0])) and \
                   not(np.isnan(self.projected_points[i,0,1])):
                    try:
                        self.depth_image[int(self.projected_points[i,0,1]),int(self.projected_points[i,0,0]),:] = 255
                    except:
                        pass
            self.depth_image_lock.release()
        except Exception as ex:
            print "Encountered an errror! ", ex
            self.depth_image_lock.release()

    def run(self):
        kernel = np.ones((3,3),'uint8')
        r = rospy.Rate(10)
        processed = set()
        while not(rospy.is_shutdown()):
            cv2.waitKey(5)

            if self.image != None and self.depth_image != None:
                self.depth_image_lock.acquire()
                if self.depth_image_timestamp not in processed:
                    nearest_image,_ = self.get_nearest_image_temporally(self.depth_image_timestamp)
                    ret, depth_threshed = cv2.threshold(self.depth_image,1,255,cv2.THRESH_BINARY)
                    combined_img = (cv2.dilate(depth_threshed,kernel)*0.5 + nearest_image*0.5).astype(dtype=np.uint8)
                    cv2.imshow("combined_feed", cv2.resize(combined_img,(self.image.shape[1]/self.downsample_factor,
                                                           self.image.shape[0]/self.downsample_factor)))

                    processed.add(self.depth_image_timestamp)
                self.depth_image_lock.release()

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('make_depth_image')
    node = DepthImageCreator()
    node.run()