/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// need to put some locking to fix some synchronization issues

#include <android/log.h>
#include <jni.h>
#include <stdlib.h>

#include <sstream>
#include <iostream>

#include <tango_support_api.h>
#include <tango_client_api.h>

#include <stdio.h>
#include <pthread.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <chrono>

#define USE_APRIL_TAGS 1
//#define BLOW_UP_TO_COLOR_DIMS 1

#include "cv2cg/apriltag/TagDetector.hpp"
#include "cv2cg/apriltag/TagFamilyFactory.hpp"



using namespace std;
using namespace cv;
using namespace std::chrono;


cv::Mat map1, map2;

float scale_factor;

#define LOG_TAG "hello-tango-jni"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

// Tango configuration file.
// Configuration file describes current states of
// Tango Service.

std::vector<cv::Ptr<april::tag::TagFamily>> families = {april::tag::TagFamilyFactory::create(april::tag::TagFamilyFactory::TAG36H11)};
april::tag::TagDetector apriltagDetector(families);

TangoConfig config;
using namespace std;

void tangoInitUndistortRectifyMap(std::vector<double> K,
                                  double D,
                                  std::vector<double> cOut,
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
            double x = scale_factor*(_x/_w - cOut[0]), y = scale_factor*(_y/_w - cOut[1]);
            double X = x / f[0];
            double Y = y / f[1];

            // the Tango uses normalized radial distances instead
            double r = sqrt(X*X + Y*Y);
            double theta_d =  1.0 / D * atan(2 * r * tan(D / 2));
            double scale = (r == 0) ? 1.0 : theta_d / r;
            double u = x*scale + c[0];
            double v = y*scale + c[1];

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
                    std::vector<double> K,
                    double D,
                    std::vector<double> cOut,
                    const Size& new_size)
{
    Size size = new_size.area() != 0 ? new_size : distorted.size();

    // TODO: we can just do this once and then apply it over and over to new images.
    // TODO: This could also potentially map to 1920x1080 image and then run through
    // Google's Tag detector.  It might be fast enough
    if (map1.empty() || map2.empty()) {
        LOGI("initializing map");
        tangoInitUndistortRectifyMap(K, D, cOut, cv::Matx33d::eye(), size, CV_16SC2, map1, map2);
    }
    remap(distorted, undistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
}

pthread_mutex_t fisheyeImageLock;

jbyteArray pixelBufferFisheye = 0;

// color camera is 1280 by 720 fish eye is 640 by 480
int imageWidthColor = 1280;			// set the values for the Yosemite devices
int imageHeightColor = 720;
int imageStrideColor = 1280;

#define IMAGE_WIDTH_FISHEYE  640
#define IMAGE_HEIGHT_FISHEYE 480

int imageStrideFisheye = 640;			// set to 640 to be compatible with Phab2 Pro
// but overwrite when you get a frame so it works in general

unsigned char rgbaUndistorted[1920*1080*3/2];
const unsigned int bufferSizeFisheyeYUVN21 = 3*768*IMAGE_HEIGHT_FISHEYE/2;		// choose 768 for stride since it is the largest I've seen
unsigned char fisheye_image_buffer_copy[bufferSizeFisheyeYUVN21];
double lastFisheyeFrameTimeStamp = 0.0;

static void onFrameAvailable(void* context, TangoCameraId camera, const TangoImageBuffer* imageBufferA) {
    // TODO: can use TangoSupport ImageBufferManager instead
    if (camera == TANGO_CAMERA_FISHEYE) {
        pthread_mutex_lock(&fisheyeImageLock);
        lastFisheyeFrameTimeStamp = imageBufferA->timestamp;
        imageStrideFisheye = imageBufferA->stride;
        memcpy(fisheye_image_buffer_copy, imageBufferA->data, imageBufferA->stride*imageBufferA->height*3/2);
        pthread_mutex_unlock(&fisheyeImageLock);
    }
}

bool TangoSetBinder(JNIEnv* env, jobject service) {
    if (TangoService_setBinder(env, service) != TANGO_SUCCESS) {
        LOGE("TangoService_setBinder(): Failed");
        return false;
    } else {
        LOGI("Successfully bound to serivice");
    }
    return true;
}

bool TangoSetConfig() {
    // Get the default TangoConfig.
    config = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
    if (config == NULL) {
        LOGE("TangoService_getConfig(): Failed");
        return false;
    }

    // disable low latency IMU .
    if (TangoConfig_setBool(config, "config_enable_low_latency_imu_integration", false) !=  TANGO_SUCCESS) {
        LOGE("turning off low latency IMU Failed");
        return false;
    }
    // disable high rate pose
    if (TangoConfig_setBool(config, "config_high_rate_pose", false) !=  TANGO_SUCCESS) {
        LOGE("turning off low latency IMU Failed");
        return false;
    }

    // disable drift corrected pose.
    if (TangoConfig_setBool(config, "config_enable_drift_correction", false) != TANGO_SUCCESS) {
        LOGE("config_enable_drift_correction");
        return false;
    } else {
        LOGI("ENABLED DRIFT CORRECTION SUCCESSFULLY");
    }
    LOGI("CONFIG SET!");

    return true;
}

bool TangoConnectCallbacks() {
    // Set listening pairs. Connect pose callback.
    // Note: the callback function should be re-connected
    // after the application resumed from background.
    if (pthread_mutex_init(&fisheyeImageLock, NULL) != 0)
    {
        LOGE("mutex init failed");
        return false;
    }

    if (TangoService_connectOnFrameAvailable(TANGO_CAMERA_FISHEYE, 0, onFrameAvailable) != TANGO_SUCCESS) {
        LOGI("frame available failed");
        return false;
    } else {
        LOGI("FISHEYE FRAME AVAILABLE WAS SUCCESSFUL!");
    }
    TangoSupport_initializeLibrary();
    return true;
}

bool TangoConnect() {
    // Connect to the Tango Service.
    // Note: connecting Tango service will start the motion
    // tracking automatically
    if (TangoService_connect(NULL, config) != TANGO_SUCCESS) {
        LOGE("TangoService_connect(): Failed");
        return false;
    }
    return true;
}

void DisconnectTango()
{
    // Disconnect Tango Service.
    TangoService_disconnect();
    // unlock all mutexes
    pthread_mutex_unlock(&fisheyeImageLock);
}

extern "C" {

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_setBinder(JNIEnv * env , jobject obj, jobject
service )
{
    TangoSetBinder(env, service ) ;
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_setupConfig(JNIEnv* env,
                                                                                  jobject obj)
{
    TangoSetConfig();

}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_connectCallbacks(JNIEnv
                                                                                       * env,
                                                                                       jobject obj)
{
    TangoConnectCallbacks();

}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_connect(JNIEnv* env,
                                                                              jobject obj
)
{
    TangoConnect();

}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_disconnect(JNIEnv
                                                                                 * env,
                                                                                 jobject obj
)
{
    DisconnectTango();

}


JNIEXPORT void
JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_returnArrayFisheye(
        JNIEnv *env, jobject This, jbyteArray pixels, jintArray stride, jdoubleArray tagDetection) {
    // this could work better across threads
    unsigned char fisheye_image_buffer_local[bufferSizeFisheyeYUVN21];
    pthread_mutex_lock(&fisheyeImageLock);
    memcpy(fisheye_image_buffer_local, fisheye_image_buffer_copy, bufferSizeFisheyeYUVN21);

    jint *st = env->GetIntArrayElements(stride, NULL);
    st[0] = imageStrideFisheye;
    env->ReleaseIntArrayElements(stride, st, 0);

    pthread_mutex_unlock(&fisheyeImageLock);

    auto start = chrono::steady_clock::now();
    TangoCameraIntrinsics fisheyeIntrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_FISHEYE, &fisheyeIntrinsics);

#ifdef USE_APRIL_TAGS
    #ifdef BLOW_UP_TO_COLOR_DIMS
        scale_factor = 0.6;
        TangoCameraIntrinsics colorIntrinsics;
        TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &colorIntrinsics);
        Size outputDimensions(colorIntrinsics.width, colorIntrinsics.height);
        std::vector<double> cOut = {colorIntrinsics.cx, colorIntrinsics.cy};
    #else
        scale_factor = 1.2;

        std::vector<double> cOut = {fisheyeIntrinsics.cx, fisheyeIntrinsics.cy};
        Size outputDimensions(fisheyeIntrinsics.width, fisheyeIntrinsics.height);
    #endif
#else
    scale_factor = 0.6;
    TangoCameraIntrinsics colorIntrinsics;

    TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &colorIntrinsics);
    Size outputDimensions(colorIntrinsics.width, colorIntrinsics.height);
    std::vector<double> cOut = {colorIntrinsics.cx, colorIntrinsics.cy};
#endif

    std::vector<double> K = {fisheyeIntrinsics.fx, fisheyeIntrinsics.cx, fisheyeIntrinsics.fy, fisheyeIntrinsics.cy};
    // pack the image into an Mat so that it can be used with the OpenCV code
    Mat testImage(fisheyeIntrinsics.height,
                  fisheyeIntrinsics.width,
                  CV_8U,
                  fisheye_image_buffer_local,
                  imageStrideFisheye);
    Mat undistortedImage;
    undistortImage(testImage, undistortedImage, K, fisheyeIntrinsics.distortion[0], cOut, outputDimensions);

#ifdef USE_APRIL_TAGS
    double tag_size = .1;           // hardcoded for now

    vector<april::tag::TagDetection> detections;
    apriltagDetector.process(undistortedImage, detections);
    auto endTags = chrono::steady_clock::now();
    jdouble *tD = env->GetDoubleArrayElements(tagDetection, NULL);
    tD[0] = -1.0;                   // this indicates that no tag was found
    for (unsigned int i = 0; i < detections.size(); i++) {
        Eigen::Matrix4d transform =
#ifdef BLOW_UP_TO_COLOR_DIMS
                // I'm not sure if I should be dividing focal length like this
                detections[i].getRelativeTransform(tag_size,
                                                   colorIntrinsics.fx / scale_factor,
                                                   colorIntrinsics.fy / scale_factor,
                                                   colorIntrinsics.cx,
                                                   colorIntrinsics.cy);
#else
                // I'm not sure if I should be dividing focal length like this
                detections[i].getRelativeTransform(tag_size,
                                                   fisheyeIntrinsics.fx / scale_factor,
                                                   fisheyeIntrinsics.fy / scale_factor,
                                                   fisheyeIntrinsics.cx,
                                                   fisheyeIntrinsics.cy);
                // this is a super hacky way to pass back the detection, but right now we are hamstrung
                // thread safety issues
                tD[0] = detections[i].p[0][0];
                tD[1] = detections[i].p[0][1];
                tD[2] = detections[i].p[1][0];
                tD[3] = detections[i].p[1][1];
                tD[4] = detections[i].p[2][0];
                tD[5] = detections[i].p[2][1];
                tD[6] = detections[i].p[3][0];
                tD[7] = detections[i].p[3][1];
#endif
        LOGI("detected %f, %f, %f", transform(0, 3), transform(1, 3), transform(2, 3));
    }
    env->ReleaseDoubleArrayElements(tagDetection, tD, 0);

    // only support this for April Tags
    jbyte * pixelJNI = env->GetByteArrayElements(pixels, NULL);
    memcpy(pixelJNI, undistortedImage.data, fisheyeIntrinsics.height*imageStrideFisheye);
    memset(pixelJNI + fisheyeIntrinsics.height*imageStrideFisheye, 128, fisheyeIntrinsics.height*imageStrideFisheye/2);
    env->ReleaseByteArrayElements(pixels, pixelJNI, 0);
#else
    for(int i = 0; i < 1920*1080; i++){
        rgbaUndistorted[i] = undistortedImage.data[i];
    }
    // use memset
    for (int i = 1920*1080; i < 1920*1080*3/2; i++) {
        rgbaUndistorted[i] = 128;
    }

    TangoImageBuffer undistortedBuffer;
    undistortedBuffer.height = 1080;
    undistortedBuffer.width = 1920;
    undistortedBuffer.timestamp = lastFisheyeFrameTimeStamp;
    undistortedBuffer.stride = 1920;
    undistortedBuffer.data = (unsigned char*) rgbaUndistorted;
    // TODO: this is not actually in this format
    undistortedBuffer.format = TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP;

    auto end = chrono::steady_clock::now();
    int timing = chrono::duration_cast<milliseconds>( end - start ).count();
    TangoSupportMarkerList list;
    TangoSupportMarkerParam param;
    param.type = TANGO_MARKER_ARTAG;
    param.marker_size = 10;
    double translation[3] = {0.0, 0.0, 0.0};
    double orientation[4] = {0.0, 0.0, 0.0, 1.0};
    TangoErrorType ret = TangoSupport_detectMarkers(&undistortedBuffer,
                                                    TANGO_CAMERA_COLOR,
                                                    translation,
                                                    orientation,
                                                    &param,
                                                    &list);
    if (ret == TANGO_SUCCESS) {
        auto endTags = chrono::steady_clock::now();
        timing += chrono::duration_cast<milliseconds>( endTags - end ).count();
        LOGI("undistort %d %d %d", timing, undistortedImage.size().width, undistortedImage.size().height);
    } else if (ret == TANGO_ERROR) {
        LOGI("undistort marker detection threw an error");
    } else {
        LOGI("undistort marker detection was invalid");
    }
#endif
}


JNIEXPORT jdouble
JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_getFisheyeFrameTimestamp(
        JNIEnv *env, jobject This) {
    double lastFisheyeFrameTimeStampCopy;

    pthread_mutex_lock(&fisheyeImageLock);
    lastFisheyeFrameTimeStampCopy = lastFisheyeFrameTimeStamp;
    pthread_mutex_unlock(&fisheyeImageLock);

    return lastFisheyeFrameTimeStampCopy;
}

}