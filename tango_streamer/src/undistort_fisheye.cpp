#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

float scale_factor = 1.25;
double distortion_coefficient = -1.0;
std::vector<double> info_vec;
image_transport::Publisher* pub_image = 0;
ros::Publisher* pub_info = 0;

using namespace cv;

void infoCallback(const sensor_msgs::CameraInfoPtr& msg) {
    if (distortion_coefficient == -1.0) {
        distortion_coefficient = msg->D[0];
        info_vec.push_back(msg->K[0]);      // fx
        info_vec.push_back(msg->K[2]);      // cx
        info_vec.push_back(msg->K[4]);      // fy
        info_vec.push_back(msg->K[5]);      // cy
    }
}

void tangoInitUndistortRectifyMap(vector<double> K,
                                  double D,
                                  InputArray R,
                                  const cv::Size& size,
                                  int m1type,
                                  OutputArray map1,
                                  OutputArray map2)
{
    CV_Assert( m1type == CV_16SC2 || m1type == CV_32F || m1type <=0 );
    map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
    map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

    CV_Assert((R.empty() || R.depth() == CV_32F || R.depth() == CV_64F));
    CV_Assert(R.empty() || R.size() == Size(3, 3) || R.total() * R.channels() == 3);

    cv::Vec2d f, c;
    f = Vec2d(K[0], K[2]);
    c = Vec2d(K[1], K[3]);

    cv::Matx33d RR  = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    cv::Matx33d PP = cv::Matx33d::eye();
    cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);

    for( int i = 0; i < size.height; ++i)
    {
        float* m1f = map1.getMat().ptr<float>(i);
        float* m2f = map2.getMat().ptr<float>(i);
        short*  m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;

        double _x = i*iR(0, 1) + iR(0, 2),
               _y = i*iR(1, 1) + iR(1, 2),
               _w = i*iR(2, 1) + iR(2, 2);

        for( int j = 0; j < size.width; ++j)
        {
            double x = scale_factor*(_x/_w - c[0]), y = scale_factor*(_y/_w-c[1]);
            double X = x / f[0];
            double Y = y / f[1];
            // the Tango uses normalized radial distances instead
            double r = sqrt(X*X + Y*Y);
            double theta_d =  1.0 / D * atan(2 * r * tan(D / 2));
            double scale = (r == 0) ? 1.0 : theta_d / r;
            //std::cout << "scale " << scale << std::endl;
            //double u = f[0]*x*scale + c[0];
            //double v = f[1]*y*scale + c[1];
            double u = x*scale + c[0];
            double v = y*scale + c[1];
            //std::cout << "(x,y)" << x << ", " << y << " (u,v) " << u <<", " << v <<std::endl;

            if( m1type == CV_16SC2 )
            {
                int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
            }
            else if( m1type == CV_32FC1 )
            {
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }

            _x += iR(0, 0);
            _y += iR(1, 0);
            _w += iR(2, 0);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// cv::fisheye::undistortImage

void undistortImage(InputArray distorted,
                    OutputArray undistorted,
                    vector<double> K,
                    double D,
                    const Size& new_size)
{
    Size size = new_size.area() != 0 ? new_size : distorted.size();

    cv::Mat map1, map2;
    tangoInitUndistortRectifyMap(K, D, cv::Matx33d::eye(), size, CV_16SC2, map1, map2 );
    remap(distorted, undistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    Mat undistorted;
    if (distortion_coefficient == -1.0 || info_vec.size() < 4 || pub_image == 0 || pub_info == 0) {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    undistortImage(cv_ptr->image, undistorted, info_vec, distortion_coefficient, Size(cv_ptr->image.size().width , cv_ptr->image.size().height));

    sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(msg->header, "bgr8", undistorted).toImageMsg();
    std::cout << "received an image " << cv_ptr->image.rows << std::endl;
    std::cout << "undistorted" << undistorted.size() << std::endl;
    pub_image->publish(msg_out);
    sensor_msgs::CameraInfo cam_info;
    cam_info.D = std::vector<double>(5, 0.0);
    cam_info.distortion_model = "plumb_bob";
    cam_info.width = cv_ptr->image.cols;
    cam_info.height = cv_ptr->image.rows;

    cam_info.K[0] = info_vec[0] / scale_factor;
    cam_info.K[2] = info_vec[1];
    cam_info.K[4] = info_vec[2] / scale_factor;
    cam_info.K[5] = info_vec[3];
    cam_info.K[8] = 1.0;


    cam_info.P[0] = info_vec[0] / scale_factor;
    cam_info.P[2] = info_vec[1];
    cam_info.P[5] = info_vec[2] / scale_factor;
    cam_info.P[6] = info_vec[3];
    cam_info.P[10] = 1.0;

    cam_info.header = msg->header;

    pub_info->publish(cam_info);
    std::cout << "published a message!" << std::endl;
}


int main(int argc, char** argv)
{
  std::string camera_name;

  ros::init(argc, argv, "undistort_fisheye");

  if (!ros::param::has("~camera_name")) {
    ROS_ERROR("Need to set a camera name");
    return -1;
  }

  ros::param::get("~camera_name", camera_name);

  ros::NodeHandle nh(camera_name, "");
  image_transport::ImageTransport it(nh);

  sensor_msgs::ImagePtr msg;
  ros::Subscriber sub = nh.subscribe("/" + camera_name + "/image_raw", 1, imageCallback);
  ros::Subscriber sub2 = nh.subscribe("/" + camera_name + "/camera_info", 1, infoCallback);
  image_transport::Publisher pub_1 = it.advertise("/fisheye_undistorted/image_raw", 10);
  pub_image = &pub_1;

  ros::Publisher pub_2 = nh.advertise<sensor_msgs::CameraInfo>("/fisheye_undistorted/camera_info", 10);
  pub_info = &pub_2;

  ros::Rate r(10);
  while (nh.ok()) {
    std::cout << "make sure images are not being updated while sending" << std::endl;
    ros::spinOnce();
    r.sleep();
  }
}