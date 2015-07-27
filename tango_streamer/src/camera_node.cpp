#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sstream> // for converting the command line parameter to integer
#include <netdb.h>
#include <netinet/in.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/asio/ip/tcp.hpp>
#include <sstream>

image_transport::CameraPublisher cam_pub;
camera_info_manager::CameraInfoManager* cinfo_manager_ = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  std::cout << "received an image" << std::endl;
  if (cinfo_manager_ != 0) {
    sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(cinfo_manager_->getCameraInfo()));
    cinfo->header.frame_id = msg->header.frame_id;
    cinfo->header.stamp = msg->header.stamp;
    cam_pub.publish(msg, cinfo);
  }
}

int main(int argc, char** argv)
{
  std::string camera_name;

  ros::init(argc, argv, "camera_node");

  if (!ros::param::has("~camera_name")) {
    ROS_ERROR("Need to set a camera name");
    return -1;
  }

  ros::param::get("~camera_name", camera_name);
  std::string camera_name_from;

  camera_name_from = camera_name;
  camera_name += "_testing";

  ros::NodeHandle nh(camera_name, "");
  image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("camera/image_raw", 10);
  cam_pub = it.advertiseCamera("image_raw", 10);

  cinfo_manager_ = new camera_info_manager::CameraInfoManager(nh, camera_name);

  sensor_msgs::ImagePtr msg;
  image_transport::Subscriber sub = it.subscribe("/" + camera_name_from + "/image_raw", 1, imageCallback);
  ros::Rate r(10);
  //cv::namedWindow("myimage");
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
