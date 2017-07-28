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

#define LOG_TAG "hello-tango-jni"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

// Tango configuration file.
// Configuration file describes current states of
// Tango Service.
TangoConfig config;

using namespace std;


pthread_mutex_t pointCloudLock;
pthread_mutex_t fisheyeImageLock;
pthread_mutex_t poseLock;
pthread_mutex_t areaPoseLock;
pthread_mutex_t colorImageLock;

jbyteArray pixelBufferColor = 0;
jbyteArray pixelBufferFisheye = 0;

// color camera is 1280 by 720 fish eye is 640 by 480
int imageWidthColor = 1280;			// set the values for the Yosemite devices
int imageHeightColor = 720;
int imageStrideColor = 1280;

#define IMAGE_WIDTH_FISHEYE  640
#define IMAGE_HEIGHT_FISHEYE 480

int imageStrideFisheye = 768;			// set to 768 to be compatible with Yosemite device,
// but overwrite when you get a frame so it works in general

// size these based on the biggest values we've seen (currently the Phab2)
const unsigned int bufferSizeColor = 4*1920*1080;
unsigned char colorCameraImageBufferRGBA[bufferSizeColor];

const unsigned int bufferSizeFisheye = 4*IMAGE_WIDTH_FISHEYE*IMAGE_HEIGHT_FISHEYE;		// no need to hard code
unsigned char fisheyeCameraImageBufferRGBA[bufferSizeFisheye];

// we keep a little extra for padding
const unsigned int bufferSizeColorYUVN21 = 3*1920*1080/2;
unsigned char color_image_buffer_copy[bufferSizeColorYUVN21];

const unsigned int bufferSizeFisheyeYUVN21 = 3*768*IMAGE_HEIGHT_FISHEYE/2;		// choose 768 for stride since it is the largest I've seen
unsigned char fisheye_image_buffer_copy[bufferSizeFisheyeYUVN21];

float xyzBuffer[4*300000];			// assume 60000 is the highest number of points, but set it higher just in case
int xyzValid = 0;
double poseBuffer[10];
double poseBufferArea[10];
double lastColorFrameTimeStamp = 0.0;
double lastFisheyeFrameTimeStamp = 0.0;
double lastFeatureTrackingErrorTimeStamp = 0.0;
int features_tracked = 0;

static void onTangoEvent(void* context, const TangoEvent* evt) {
    if (strcmp(evt->event_key, "TooFewFeaturesTracked") == 0) {
        lastFeatureTrackingErrorTimeStamp = evt->timestamp;
        features_tracked = atoi(evt->event_value);
    }
}

static void onFrameAvailable(void* context, TangoCameraId camera, const TangoImageBuffer* imageBufferA) {
    // TODO: can use TangoSupport ImageBufferManager instead
    if (camera == TANGO_CAMERA_FISHEYE) {
        pthread_mutex_lock(&fisheyeImageLock);
        lastFisheyeFrameTimeStamp = imageBufferA->timestamp;
        imageStrideFisheye = imageBufferA->stride;
        memcpy(fisheye_image_buffer_copy, imageBufferA->data, imageBufferA->stride*imageBufferA->height*3/2);
        pthread_mutex_unlock(&fisheyeImageLock);
    } else {
        pthread_mutex_lock(&colorImageLock);
        lastColorFrameTimeStamp = imageBufferA->timestamp;
        memcpy(color_image_buffer_copy, imageBufferA->data, imageBufferA->stride*imageBufferA->height*3/2);
        imageWidthColor = imageBufferA->width;
        imageHeightColor = imageBufferA->height;
        pthread_mutex_unlock(&colorImageLock);
    }
}


static void onPointCloudAvailable(void* context, const TangoPointCloud* pointCloud) {
    pthread_mutex_lock(&pointCloudLock);
    xyzValid = pointCloud->num_points;
    xyzBuffer[0] = pointCloud->timestamp;
    memcpy(xyzBuffer + 1, pointCloud->points, 4 * sizeof(float) * xyzValid);
    pthread_mutex_unlock(&pointCloudLock);
}

static void onPoseAvailable(void* context, const TangoPoseData* pose) {
    if (pose->frame.base == TANGO_COORDINATE_FRAME_START_OF_SERVICE &&
        pose->frame.target == TANGO_COORDINATE_FRAME_DEVICE) {
        pthread_mutex_lock(&poseLock);
        poseBuffer[0] = pose->translation[0];
        poseBuffer[1] = pose->translation[1];
        poseBuffer[2] = pose->translation[2];
        poseBuffer[3] = pose->orientation[0];
        poseBuffer[4] = pose->orientation[1];
        poseBuffer[5] = pose->orientation[2];
        poseBuffer[6] = pose->orientation[3];
        poseBuffer[7] = pose->timestamp; // the last is the time
        poseBuffer[8] = pose->status_code;

        // TODO: might need to communicate this independently of the pose (in case callbacks stop happening)
        if (fabs(pose->timestamp - lastFeatureTrackingErrorTimeStamp) < 1.0) {
            poseBuffer[9] = features_tracked;
        } else {
            poseBuffer[9] = -1.0;
        }
        pthread_mutex_unlock(&poseLock);

    } else if (pose->frame.base == TANGO_COORDINATE_FRAME_AREA_DESCRIPTION &&
               pose->frame.target == TANGO_COORDINATE_FRAME_DEVICE) {
        pthread_mutex_lock(&areaPoseLock);
        poseBufferArea[0] = pose->translation[0];
        poseBufferArea[1] = pose->translation[1];
        poseBufferArea[2] = pose->translation[2];
        poseBufferArea[3] = pose->orientation[0];
        poseBufferArea[4] = pose->orientation[1];
        poseBufferArea[5] = pose->orientation[2];
        poseBufferArea[6] = pose->orientation[3];
        poseBufferArea[7] = pose->timestamp; // the last is the time
        poseBufferArea[8] = pose->status_code;

        if (fabs(pose->timestamp - lastFeatureTrackingErrorTimeStamp) < 1.0) {
            poseBufferArea[9] = features_tracked;
        } else {
            poseBufferArea[9] = -1.0;
        }
        pthread_mutex_unlock(&areaPoseLock);

        LOGI("GOT A POSE!! %f", poseBufferArea[7]);
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

bool TangoSetConfig(bool streamColorImages) {
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
        LOGE("turning off high rate pose Failed");
        return false;
    }
    // Enable depth.
    if (TangoConfig_setBool(config, "config_enable_depth", true) !=  TANGO_SUCCESS) {
        LOGE("config_enable_depth Failed");
        return false;
    }

    // Set depth mode (note: this is not needed on newer SDKs)
    if (TangoConfig_setInt32(config, "config_depth_mode", TANGO_POINTCLOUD_XYZC) !=  TANGO_SUCCESS) {
        LOGE("PAUL: couldn't set the depth mode Failed");
        return false;
    }

    // Enable camera.
    if (TangoConfig_setBool(config, "config_enable_color_camera", streamColorImages) != TANGO_SUCCESS) {
        LOGE("config_enable_color Failed");
        return false;
    } else {
        LOGI("ENABLED COLOR CAMERA SUCCESSFULLY %d", streamColorImages);
    }

    // disable drift corrected pose.
    if (TangoConfig_setBool(config, "config_enable_drift_correction", false) != TANGO_SUCCESS) {
        LOGE("config_enable_drift_correction");
        return false;
    } else {
        LOGI("ENABLED DRIFT CORRECTION SUCCESSFULLY %d", streamColorImages);
    }
    LOGI("CONFIG SET!");

    return true;
}

bool TangoConnectCallbacks(bool streamColorImages) {
    // Set listening pairs. Connect pose callback.
    // Note: the callback function should be re-connected
    // after the application resumed from background.
    LOGI("Starting to setup the  mutexes");
    if (pthread_mutex_init(&pointCloudLock, NULL) != 0)
    {
        LOGE("mutex init failed");
        return false;
    }
    if (pthread_mutex_init(&fisheyeImageLock, NULL) != 0)
    {
        LOGE("mutex init failed");
        return false;
    }
    if (pthread_mutex_init(&poseLock, NULL) != 0)
    {
        LOGE("mutex init failed");
        return false;
    }
    if (pthread_mutex_init(&areaPoseLock, NULL) != 0)
    {
        LOGE("mutex init failed");
        return false;
    }
    if (pthread_mutex_init(&colorImageLock, NULL) != 0)
    {
        LOGE("mutex init failed");
        return false;
    }

    if (TangoService_connectOnTangoEvent(onTangoEvent) != TANGO_SUCCESS) {
        LOGI("TANGO EVENT FAILED");
    }

    if (TangoService_connectOnFrameAvailable(TANGO_CAMERA_FISHEYE, 0, onFrameAvailable) != TANGO_SUCCESS) {
        LOGI("frame available failed");
        return false;
    } else {
        LOGI("FISHEYE FRAME AVAILABLE WAS SUCCESSFUL!");
    }

    if (streamColorImages) {
        LOGI("PAUL: setting up color frame callbacks");
        if (TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, 0, onFrameAvailable) !=
            TANGO_SUCCESS) {
            LOGI("frame available failed");
            return false;
        } else {
            LOGI("COLOR FRAME AVAILABLE WAS SUCCESSFUL!");
        }
    }
    LOGI("PAUL: about to connect to pointclouds");
    // Attach the onPointCloudAvailable callback.
    if (TangoService_connectOnPointCloudAvailable(onPointCloudAvailable) != TANGO_SUCCESS) {
        LOGI("PAUL: TangoService_connectOnPointCloudAvailable(): Failed");
        return false;
    } else {
        LOGI("PAUL: Sucessfully connected point clouds");
    }

    TangoCoordinateFramePair* pairs = (TangoCoordinateFramePair*)malloc(2*sizeof(TangoCoordinateFramePair));
    pairs[0].base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    pairs[0].target = TANGO_COORDINATE_FRAME_DEVICE;
    pairs[1].base = TANGO_COORDINATE_FRAME_AREA_DESCRIPTION;
    pairs[1].target = TANGO_COORDINATE_FRAME_DEVICE;
    // Currently cannot get the pose of the depth camera
    if (TangoService_connectOnPoseAvailable(2, pairs, onPoseAvailable) != TANGO_SUCCESS) {
        LOGI("TangoService_connectOnPoseAvailable(): Failed");
        return false;
    } else {
        LOGI("Successfully connected pose callbacks");
    }
//	TangoSupport_initializeLibrary();
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
    pthread_mutex_unlock(&colorImageLock);
    pthread_mutex_unlock(&pointCloudLock);
    pthread_mutex_unlock(&fisheyeImageLock);
    pthread_mutex_unlock(&poseLock);
    pthread_mutex_unlock(&areaPoseLock);
    pthread_mutex_unlock(&colorImageLock);


    // make sure all mutexes are reset
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
                                                                                  jobject obj, jboolean streamColorImages)
{
    TangoSetConfig(streamColorImages);

}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_connectCallbacks(JNIEnv
                                                                                       * env,
                                                                                       jobject obj, jboolean streamColorImages)
{
    TangoConnectCallbacks(streamColorImages);

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

JNIEXPORT jbyteArray

JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_returnArrayColor(JNIEnv *env,
                                                                                               jobject This,
                                                                                               jdoubleArray frameTimeStamp,
                                                                                               jintArray dimensions) {
    pixelBufferColor = env->NewByteArray(bufferSizeColorYUVN21);
    pthread_mutex_lock(&colorImageLock);
    jdouble *ts = env->GetDoubleArrayElements(frameTimeStamp, NULL);
    ts[0] = lastColorFrameTimeStamp;
    env->ReleaseDoubleArrayElements(frameTimeStamp, ts, 0);

    jint *dims = env->GetIntArrayElements(dimensions, NULL);
    dims[0] = imageWidthColor;
    dims[1] = imageHeightColor;
    env->ReleaseIntArrayElements(dimensions, dims, 0);

    env->SetByteArrayRegion(pixelBufferColor, 0, bufferSizeColorYUVN21,
                            reinterpret_cast<jbyte *>(color_image_buffer_copy));
    pthread_mutex_unlock(&colorImageLock);

    return pixelBufferColor;
}

JNIEXPORT jdoubleArray
JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_getPoses(
        JNIEnv *env, jobject This, jdoubleArray poseTimeStamps) {
    jint len = env->GetArrayLength(poseTimeStamps);
    jdouble *ts = env->GetDoubleArrayElements(poseTimeStamps, NULL);
    jdoubleArray posesDoubleArray = env->NewDoubleArray(10*len);
    TangoErrorType ret;
    TangoCoordinateFramePair sofs_device;
    TangoPoseData pose;
    double* queried_poses = new double[len*10];

    sofs_device.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    sofs_device.target = TANGO_COORDINATE_FRAME_DEVICE;

    for (int i = 0; i < len; i++) {
        ret = TangoService_getPoseAtTime(
                ts[i],
                sofs_device,
                &pose);
        if (ret != TANGO_SUCCESS || pose.status_code != TANGO_POSE_VALID) {
            queried_poses[i*10 + 8] = (double)TANGO_POSE_INVALID;			// indicates an invalid pose
            // the rest of the pose is undefined
        } else {
            queried_poses[i*10 + 0] = pose.translation[0];
            queried_poses[i*10 + 1] = pose.translation[1];
            queried_poses[i*10 + 2] = pose.translation[2];
            queried_poses[i*10 + 3] = pose.orientation[0];
            queried_poses[i*10 + 4] = pose.orientation[1];
            queried_poses[i*10 + 5] = pose.orientation[2];
            queried_poses[i*10 + 6] = pose.orientation[3];
            queried_poses[i*10 + 7] = (double)pose.timestamp;
            queried_poses[i*10 + 8] = (double)pose.status_code;
            queried_poses[i*10 + 9] = -1.0;		// kludge to make this similar to the returnPose API
        }
    }
    env->ReleaseDoubleArrayElements(poseTimeStamps, ts, 0);
    env->SetDoubleArrayRegion(posesDoubleArray, 0, len*10, queried_poses);
    delete[] queried_poses;
    return posesDoubleArray;
}

JNIEXPORT jbyteArray
JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_returnArrayFisheye(
        JNIEnv *env, jobject This, jdoubleArray frameTimeStamp, jintArray stride) {
    pixelBufferFisheye = env->NewByteArray(bufferSizeFisheyeYUVN21);
    pthread_mutex_lock(&fisheyeImageLock);
    jdouble *ts = env->GetDoubleArrayElements(frameTimeStamp, NULL);
    ts[0] = lastFisheyeFrameTimeStamp;
    env->ReleaseDoubleArrayElements(frameTimeStamp, ts, 0);

    jint *st = env->GetIntArrayElements(stride, NULL);
    st[0] = imageStrideFisheye;
    env->ReleaseIntArrayElements(stride, st, 0);

    env->SetByteArrayRegion(pixelBufferFisheye, 0, bufferSizeFisheyeYUVN21,
                            reinterpret_cast<jbyte *>(fisheye_image_buffer_copy));
    pthread_mutex_unlock(&fisheyeImageLock);

    return pixelBufferFisheye;
}

JNIEXPORT jdouble

JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_getColorFrameTimestamp(
        JNIEnv *env, jobject This) {
    double lastColorFrameTimeStampCopy;

    pthread_mutex_lock(&colorImageLock);
    lastColorFrameTimeStampCopy = lastColorFrameTimeStamp;
    pthread_mutex_unlock(&colorImageLock);

    return lastColorFrameTimeStampCopy;
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

JNIEXPORT jdoubleArray

JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_returnPoseArray(JNIEnv *env,
                                                                                              jobject This) {
    jdoubleArray poseDoubleArray = env->NewDoubleArray(10);
    pthread_mutex_lock(&poseLock);
    env->SetDoubleArrayRegion(poseDoubleArray, 0, 10, poseBuffer);
    pthread_mutex_unlock(&poseLock);
    return poseDoubleArray;
}


JNIEXPORT jdoubleArray

JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_returnPoseAreaArray(
        JNIEnv *env, jobject This) {
    jdoubleArray poseDoubleArray = env->NewDoubleArray(10);
    pthread_mutex_lock(&areaPoseLock);
    env->SetDoubleArrayRegion(poseDoubleArray, 0, 10, poseBufferArea);
    pthread_mutex_unlock(&areaPoseLock);
    return poseDoubleArray;
}

JNIEXPORT jdoubleArray

JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_returnIntrinsicsColor(
        JNIEnv *env, jobject This) {
    double intrinsicsArray[11];
    jdoubleArray intrinsicsDoubleArray = env->NewDoubleArray(11);
    TangoCameraIntrinsics ccIntrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &ccIntrinsics);

    //	TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &ccIntrinsics);

    intrinsicsArray[0] = ccIntrinsics.width;
    intrinsicsArray[1] = ccIntrinsics.height;
    intrinsicsArray[2] = ccIntrinsics.fx;
    intrinsicsArray[3] = ccIntrinsics.fy;
    intrinsicsArray[4] = ccIntrinsics.cx;
    intrinsicsArray[5] = ccIntrinsics.cy;
    intrinsicsArray[6] = ccIntrinsics.distortion[0];
    intrinsicsArray[7] = ccIntrinsics.distortion[1];
    intrinsicsArray[8] = ccIntrinsics.distortion[2];
    intrinsicsArray[9] = ccIntrinsics.distortion[3];
    intrinsicsArray[10] = ccIntrinsics.distortion[4];

    env->SetDoubleArrayRegion(intrinsicsDoubleArray, 0, 11, intrinsicsArray);
    return intrinsicsDoubleArray;
}

JNIEXPORT jdoubleArray

JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_returnIntrinsicsFisheye(
        JNIEnv *env, jobject This) {
    double intrinsicsArray[11];
    jdoubleArray intrinsicsDoubleArray = env->NewDoubleArray(11);
    TangoCameraIntrinsics ccIntrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_FISHEYE, &ccIntrinsics);

    //	TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &ccIntrinsics);

    intrinsicsArray[0] = ccIntrinsics.width;
    intrinsicsArray[1] = ccIntrinsics.height;
    intrinsicsArray[2] = ccIntrinsics.fx;
    intrinsicsArray[3] = ccIntrinsics.fy;
    intrinsicsArray[4] = ccIntrinsics.cx;
    intrinsicsArray[5] = ccIntrinsics.cy;
    intrinsicsArray[6] = ccIntrinsics.distortion[0];
    intrinsicsArray[7] = ccIntrinsics.distortion[1];
    intrinsicsArray[8] = ccIntrinsics.distortion[2];
    intrinsicsArray[9] = ccIntrinsics.distortion[3];
    intrinsicsArray[10] = ccIntrinsics.distortion[4];

    env->SetDoubleArrayRegion(intrinsicsDoubleArray, 0, 11, intrinsicsArray);
    return intrinsicsDoubleArray;
}

JNIEXPORT jfloatArray
JNICALL Java_com_projecttango_examples_cpp_hellomotiontracking_TangoJniNative_returnPointCloud(JNIEnv *env,
                                                                                               jobject This) {
    pthread_mutex_lock(&pointCloudLock);
    jfloatArray pointCloudFloatArray = env->NewFloatArray(xyzValid * 4 + 1);
    env->SetFloatArrayRegion(pointCloudFloatArray, 0, xyzValid * 4 + 1, xyzBuffer);
    pthread_mutex_unlock(&pointCloudLock);

    return pointCloudFloatArray;
}

}
