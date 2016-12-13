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

#include <tango_support_api.h>
//#include <tango_client_api.h>

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

pthread_mutex_t pointCloudLock;
pthread_mutex_t fisheyeImageLock;
pthread_mutex_t poseLock;
pthread_mutex_t areaPoseLock;
pthread_mutex_t colorImageLock;

jbyteArray pixelBufferColor = 0;
jbyteArray pixelBufferFisheye = 0;

// color camera is 1280 by 720 fish eye is 640 by 480
#define IMAGE_WIDTH_COLOR  1280
#define IMAGE_HEIGHT_COLOR 720

#define IMAGE_WIDTH_FISHEYE  640
#define IMAGE_HEIGHT_FISHEYE 480
#define IMAGE_STRIDE_FISHEYE 768

// TODO make these  class
const unsigned int bufferSizeColor = 4*IMAGE_WIDTH_COLOR*IMAGE_HEIGHT_COLOR;		// no need to hard code
unsigned char colorCameraImageBufferRGBA[4*IMAGE_WIDTH_COLOR*IMAGE_HEIGHT_COLOR];

const unsigned int bufferSizeFisheye = 4*IMAGE_WIDTH_FISHEYE*IMAGE_HEIGHT_FISHEYE;		// no need to hard code
unsigned char fisheyeCameraImageBufferRGBA[4*IMAGE_WIDTH_FISHEYE*IMAGE_HEIGHT_FISHEYE];

// we keep a little extra for padding
unsigned char color_image_buffer_copy[3*IMAGE_WIDTH_COLOR*IMAGE_HEIGHT_COLOR];
unsigned char fisheye_image_buffer_copy[3*IMAGE_WIDTH_FISHEYE*IMAGE_HEIGHT_FISHEYE];

float xyzBuffer[3*60000];			// assume 60000 is the highest number of points
int xyzValid = 0;
double poseBuffer[10];
double poseBufferArea[10];
double lastColorFrameTimeStamp = 0.0;
double lastFisheyeFrameTimeStamp = 0.0;
double lastFeatureTrackingErrorTimeStamp = 0.0;
int features_tracked = 0;

void convertToRGBAWithStride(int IMAGE_STRIDE, int W, int H, unsigned char* image_buffer_copy, unsigned char* cameraImageBufferRGBA) {;
	int y, v, u, R, G, B;
	// sizes of Y, U, and V pixel arrays.
	const int sizeOfYDataL   = IMAGE_STRIDE * H;
	int curr = 0;
	// indices, marking the begin of the y, u, and v data in the pixel buffer.
	const int beginOfYDataL  = 0;
	const int beginOfUVDataL  = sizeOfYDataL;

	// YUV, Y, and UV pixel sub arrays.
	const unsigned char*  yuvArrL     = image_buffer_copy;
	const unsigned char*  yArrL       = &yuvArrL[ beginOfYDataL  ];
	const unsigned char*  uvArrL      = &yuvArrL[ beginOfUVDataL ];

	// --- local variables ------------------------------

	// image pixel coordinates.
	int xL,yL;
	// halved image pixel coordinates.
	int hxL,hyL;
	// ARGB value.
	int argbL;
	// --------------------------------------------------

	// translate YUV NV21 -> ARGB, using
	//
	//      / R \   / 1.000   0.000   1.596 \   /   Y   \
	//      | G | = | 1.000  -0.391  -0.813 | * | U-128 |
	//      \ B /   \ 1.000   2.018   0.000 /   \ V-128 /
	//

	for( yL=1,hyL=0; yL<H; yL++,hyL=yL>>1 )
	{
	  for( xL=0,hxL=0; xL<W; xL++,hxL=xL>>1 )
	  {
		  // use stride instead of W (hard code to 768)
		y = (int)( yArrL [  yL*IMAGE_STRIDE +    xL   ] )      ;
		v = (int)( uvArrL[ hyL*IMAGE_STRIDE + 2*hxL   ] ) - 128;
		u = (int)( uvArrL[ hyL*IMAGE_STRIDE + 2*hxL+1 ] ) - 128;

		R = (int)( y               + ( 1.596f*v) );
		G = (int)( y + (-0.391f*u) + (-0.813f*v) );
		B = (int)( y + ( 2.018f*u)               );

		// clip RGB values to [0..255].
		R = R < 0 ? 0 : (R > 255 ? 255 : R);
		G = G < 0 ? 0 : (G > 255 ? 255 : G);
		B = B < 0 ? 0 : (B > 255 ? 255 : B);

		// combine to ARGB value.
		argbL = 0xff000000 | (B << 16) | (G << 8) | R;

//		argbL = 0xff000000 | (R << 16) | (G << 8) | B;
		((int*)cameraImageBufferRGBA)[curr++] = argbL;
	  } // for
	} // for
} // function

//
//void convertToRGBA() {
//	// image width and height
//	const int W               = IMAGE_WIDTH;
//	const int H               = IMAGE_HEIGHT;
//	int y, v, u, R, G, B;
//	// sizes of Y, U, and V pixel arrays.
//	const int sizeOfYDataL   = W * H;
//	int curr = 0;
//	// indices, marking the begin of the y, u, and v data in the pixel buffer.
//	const int beginOfYDataL  = 0;
//	const int beginOfUVDataL  = sizeOfYDataL;
//
//	// YUV, Y, and UV pixel sub arrays.
//	const unsigned char*  yuvArrL     = image_buffer_copy;
//	const unsigned char*  yArrL       = &yuvArrL[ beginOfYDataL  ];
//	const unsigned char*  uvArrL      = &yuvArrL[ beginOfUVDataL ];
//
//	// --- local variables ------------------------------
//
//	// image pixel coordinates.
//	int xL,yL;
//	// halved image pixel coordinates.
//	int hxL,hyL;
//	// ARGB value.
//	int argbL;
//	// --------------------------------------------------
//
//	// translate YUV NV21 -> ARGB, using
//	//
//	//      / R \   / 1.000   0.000   1.596 \   /   Y   \
//	//      | G | = | 1.000  -0.391  -0.813 | * | U-128 |
//	//      \ B /   \ 1.000   2.018   0.000 /   \ V-128 /
//	//
//
//	for( yL=1,hyL=0; yL<H; yL++,hyL=yL>>1 )
//	{
//	  for( xL=0,hxL=0; xL<W; xL++,hxL=xL>>1 )
//	  {
//		  // use stride instead of W (hard code to 768)
//		y = (int)( yArrL [  yL*W +    xL   ] )      ;
//		v = (int)( uvArrL[ hyL*W + 2*hxL   ] ) - 128;
//		u = (int)( uvArrL[ hyL*W + 2*hxL+1 ] ) - 128;
//
//		u = 0;
//		v = 0;
//
//		R = (int)( y               + ( 1.596f*v) );
//		G = (int)( y + (-0.391f*u) + (-0.813f*v) );
//		B = (int)( y + ( 2.018f*u)               );
//
//		// clip RGB values to [0..255].
//		R = R < 0 ? 0 : (R > 255 ? 255 : R);
//		G = G < 0 ? 0 : (G > 255 ? 255 : G);
//		B = B < 0 ? 0 : (B > 255 ? 255 : B);
//
//		// combine to ARGB value.
//		argbL = 0xff000000 | (B << 16) | (G << 8) | R;
//
////		argbL = 0xff000000 | (R << 16) | (G << 8) | B;
//		((int*)cameraImageBufferRGBA)[curr++] = argbL;
//	  } // for
//	} // for
//} // function

static void onTangoEvent(void* context, const TangoEvent* evt) {
	if (strcmp(evt->event_key, "TooFewFeaturesTracked") == 0) {
		lastFeatureTrackingErrorTimeStamp = evt->timestamp;
		features_tracked = atoi(evt->event_value);
	}
}

static void onFrameAvailable(void* context, TangoCameraId camera, const TangoImageBuffer* imageBufferA) {

		if (camera == TANGO_CAMERA_FISHEYE) {
			pthread_mutex_lock(&fisheyeImageLock);
			lastFisheyeFrameTimeStamp = imageBufferA->timestamp;
			memcpy(fisheye_image_buffer_copy, imageBufferA->data, imageBufferA->stride*imageBufferA->height*3/2);
			pthread_mutex_unlock(&fisheyeImageLock);
		} else {
			pthread_mutex_lock(&colorImageLock);
			lastColorFrameTimeStamp = imageBufferA->timestamp;
			memcpy(color_image_buffer_copy, imageBufferA->data, imageBufferA->stride*imageBufferA->height*3/2);
			pthread_mutex_unlock(&colorImageLock);
		}
	  // --- local constants ------------------------------

		//memcpy(image_buffer_copy, imageBufferA->data, imageBufferA->width*imageBufferA->height*3/2);
}

static void onPointCloudAvailable(void* context, const TangoPointCloud* XYZ_ij) {
	// experiment with putting everything in the odom frame
	TangoPointCloud transformed;
	TangoMatrixTransformData depth_cam_to_start_of_service;
	int i;

	transformed.points = malloc(sizeof(float) * XYZ_ij->num_points * 4);

//	if (TangoSupport_transformPointCloud(depth_cam_to_start_of_service.matrix, XYZ_ij, &transformed) != TANGO_SUCCESS) {
//		LOE("Failed to transform point cloud");
//	}
	// clear out any old points
	LOGI("PCL: processing");
	pthread_mutex_lock(&pointCloudLock);

	xyzValid = XYZ_ij->num_points;
	for (i = 0; i < 3*60000; i++) {
		xyzBuffer[i] = 0;
	}
	xyzBuffer[0] = XYZ_ij->timestamp;
	for (i = 1; i < XYZ_ij->num_points; i++) {
		xyzBuffer[(i-1)*3+1] = XYZ_ij->points[i][0];
		xyzBuffer[(i-1)*3+2] = XYZ_ij->points[i][1];
		xyzBuffer[(i-1)*3+3] = XYZ_ij->points[i][2];
	}
	pthread_mutex_unlock(&pointCloudLock);

}

static void onPoseAvailable(void* context, const TangoPoseData* pose) {
//	TangoPoseData startOfServiceInIMU;
//	TangoCoordinateFramePair startOfServiceToIMU;
//	startOfServiceToIMU.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
//	startOfServiceToIMU.target =  TANGO_COORDINATE_FRAME_IMU;
//
//	TangoService_getPoseAtTime(pose->timestamp, startOfServiceToIMU, &startOfServiceInIMU);
//	LOGI("startOfServiceInIMU Frame Fixed Position: %f, %f, %f. Orientation: %f, %f, %f, %f, %f",
//		 startOfServiceInIMU.translation[0], startOfServiceInIMU.translation[1], startOfServiceInIMU.translation[2],
//		 startOfServiceInIMU.orientation[0], startOfServiceInIMU.orientation[1], startOfServiceInIMU.orientation[2],
//		 startOfServiceInIMU.orientation[3], startOfServiceInIMU.timestamp);
//
//    TangoPoseData deviceInIMUFrame;
//    TangoCoordinateFramePair deviceToIMU;
//    deviceToIMU.base = TANGO_COORDINATE_FRAME_IMU;
//    deviceToIMU.target =  TANGO_COORDINATE_FRAME_DEVICE;
//
//	TangoService_getPoseAtTime(pose->timestamp, deviceToIMU, &deviceInIMUFrame);
//	LOGI("DeviceInIMU Frame Fixed Position: %f, %f, %f. Orientation: %f, %f, %f, %f, %f",
//		 deviceInIMUFrame.translation[0], deviceInIMUFrame.translation[1], deviceInIMUFrame.translation[2],
//		 deviceInIMUFrame.orientation[0], deviceInIMUFrame.orientation[1], deviceInIMUFrame.orientation[2],
//		 deviceInIMUFrame.orientation[3], deviceInIMUFrame.timestamp);
//
//	TangoPoseData depthCamInIMUFrame;
//	TangoCoordinateFramePair depthCamToIMU;
//
//	depthCamToIMU.base = TANGO_COORDINATE_FRAME_IMU;
//	depthCamToIMU.target =  TANGO_COORDINATE_FRAME_CAMERA_FISHEYE;
//
//	TangoService_getPoseAtTime(pose->timestamp, depthCamToIMU, &depthCamInIMUFrame);
//	LOGI("FisheyeCamInIMU Fixed Position: %f, %f, %f. Orientation: %f, %f, %f, %f, %f",
//		 depthCamInIMUFrame.translation[0], depthCamInIMUFrame.translation[1], depthCamInIMUFrame.translation[2],
//		 depthCamInIMUFrame.orientation[0], depthCamInIMUFrame.orientation[1], depthCamInIMUFrame.orientation[2],
//		 depthCamInIMUFrame.orientation[3], depthCamInIMUFrame.timestamp);
	if (pose->frame.base == TANGO_COORDINATE_FRAME_START_OF_SERVICE &&
		pose->frame.target == TANGO_COORDINATE_FRAME_DEVICE) {
		// experiment with using the IMU instead of the device
		//pose = &startOfServiceInIMU;
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
		pthread_mutex_unlock(&poseLock);

		// TODO: might need to communicate this independently of the pose (in case callbacks stop happening)
		if (fabs(pose->timestamp - lastFeatureTrackingErrorTimeStamp) < 1.0) {
			poseBuffer[9] = features_tracked;
		} else {
			poseBuffer[9] = -1.0;
		}
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
		pthread_mutex_unlock(&areaPoseLock);

		if (fabs(pose->timestamp - lastFeatureTrackingErrorTimeStamp) < 1.0) {
			poseBufferArea[9] = features_tracked;
		} else {
			poseBufferArea[9] = -1.0;
		}
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

  // Enable depth.
  if (TangoConfig_setBool(config, "config_enable_depth", true) !=  TANGO_SUCCESS) {
    LOGE("config_enable_depth Failed");
    return false;
  }
	if (TangoConfig_setInt32( config, "config_depth_mode", TANGO_POINTCLOUD_XYZC ) != TANGO_SUCCESS) {
		LOGE("Failed to set the new depth mode");
		return false;
	} else {
		LOGI("successfully using new depth mode");
	}

//  if (TangoConfig_setBool(config, "config_enable_learning_mode", true)
//      != TANGO_SUCCESS) {
//    // Handle the error.
//	  LOGE("AREA LEARNING FAILED");
//  } else {
//	  LOGI("AREA LEARING SUCCESSFUL");
//  }

  // Enable camera.
  if (TangoConfig_setBool(config, "config_enable_color_camera", true) != TANGO_SUCCESS) {
    LOGE("config_enable_depth Failed");
    return false;
  } else {
	  LOGI("ENABLED COLOR CAMERA SUCCESSFULLY");
  }
  // Enable drift correction.
//  if (TangoConfig_setBool(config, "config_enable_drift_correction", true) != TANGO_SUCCESS) {
//	  LOGE("config_enable_drift_correction Failed");
//	  return false;
//  } else {
//	  LOGI("ENABLED DRIFT CORRECTION SUCCESSFULLY");
//  }

//	// this seems to make things less accurate when it is turned on
//	if (TangoConfig_setBool(config, "config_enable_low_latency_imu_integration", false) != TANGO_SUCCESS) {
//		LOGE("config_enable_low_latency_imu_integration Failed");
//	} else {
//		LOGI("SUCCESSFULLY DISABLED LOW LATENCY POSE");
//	}
//	// this seems to make things less accurate when it is turned on
//	if (TangoConfig_setBool(config, "config_high_rate_pose", false) != TANGO_SUCCESS) {
//		LOGE("config_high_rate_pose Failed");
//		return false;
//	} else {
//		LOGI("SUCCESSFULLY DISABLED HIGH RATE POSE");
//	}
//	// this seems to make things less accurate when it is turned on
//	if (TangoConfig_setBool(config, "config_smooth_pose", false) != TANGO_SUCCESS) {
//		LOGE("config_smooth_pose Failed");
//		return false;
//	} else {
//		LOGI("SUCCESSFULLY DISABLED POSE SMOOTHING");
//	}
  return true;
}

bool TangoConnectCallbacks() {
  // Set listening pairs. Connect pose callback.
  // Note: the callback function should be re-connected 
  // after the application resumed from background.

  if (TangoService_connectOnTangoEvent(onTangoEvent) != TANGO_SUCCESS) {
		LOGI("TANGO EVENT FAILED");
  }

  if (TangoService_connectOnFrameAvailable(TANGO_CAMERA_FISHEYE, 0, onFrameAvailable) != TANGO_SUCCESS) {
	LOGI("frame available failed");
	return false;
  } else {
	 LOGI("FRAME AVAILABLE WAS SUCCESSFUL!");
  }

  if (TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, 0, onFrameAvailable) != TANGO_SUCCESS) {
	LOGI("frame available failed");
	return false;
  } else {
	 LOGI("FRAME AVAILABLE WAS SUCCESSFUL!");
  }

  // Attach the onXYZijAvailable callback.
  if (TangoService_connectOnPointCloudAvailable(onPointCloudAvailable) != TANGO_SUCCESS) {
	  LOGI("TangoService_connectOnPointCloudAvailable(): Failed");
	  return false;
  } else {
	  LOGI("Sucessfully connected point clouds");
  }

  TangoCoordinateFramePair* pairs = (TangoCoordinateFramePair*)malloc(2*sizeof(TangoCoordinateFramePair));
  pairs[0].base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pairs[0].target = TANGO_COORDINATE_FRAME_DEVICE;

  pairs[1].base = TANGO_COORDINATE_FRAME_AREA_DESCRIPTION;
  pairs[1].target = TANGO_COORDINATE_FRAME_DEVICE;
  // Currently cannot get the pose of the depth camera
  if (TangoService_connectOnPoseAvailable(2, pairs, onPoseAvailable)
      != TANGO_SUCCESS) {
    LOGI("TangoService_connectOnPoseAvailable(): Failed");
    return false;
  }
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
}

JNIEXPORT void JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_setBinder(JNIEnv* env, jobject obj, jobject service)
{
  TangoSetBinder(env, service);
}

JNIEXPORT void JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_setupConfig(JNIEnv* env, jobject obj, jobject activity)
{
  TangoSetConfig();
}

JNIEXPORT void JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_connectCallbacks(JNIEnv* env, jobject obj, jobject activity)
{
  TangoConnectCallbacks();
}

JNIEXPORT void JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_connect(JNIEnv* env, jobject obj)
{
  TangoConnect();
}

JNIEXPORT void JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_disconnect(JNIEnv* env, jobject obj)
{
  DisconnectTango();
}

JNIEXPORT jbyteArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnArrayColor(JNIEnv *env, jobject This, jdoubleArray frameTimeStamp)
{
	pixelBufferColor = (*env)->NewByteArray(env, bufferSizeColor);
	pthread_mutex_lock(&colorImageLock);
	jdouble* ts = (*env)->GetDoubleArrayElements(env, frameTimeStamp,NULL);
    ts[0] = lastColorFrameTimeStamp;
	(*env)->ReleaseDoubleArrayElements(env, frameTimeStamp, ts, 0);
	convertToRGBAWithStride(IMAGE_WIDTH_COLOR, IMAGE_WIDTH_COLOR, IMAGE_HEIGHT_COLOR, color_image_buffer_copy, colorCameraImageBufferRGBA);
	(*env)->SetByteArrayRegion (env, pixelBufferColor, 0, bufferSizeColor, colorCameraImageBufferRGBA);
	pthread_mutex_unlock(&colorImageLock);

	return pixelBufferColor;
}

JNIEXPORT jbyteArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnArrayFisheye(JNIEnv *env, jobject This, jdoubleArray frameTimeStamp)
{
	pixelBufferFisheye = (*env)->NewByteArray(env, bufferSizeFisheye);
	pthread_mutex_lock(&fisheyeImageLock);
	jdouble* ts = (*env)->GetDoubleArrayElements(env, frameTimeStamp,NULL);
	ts[0] = lastFisheyeFrameTimeStamp;
	(*env)->ReleaseDoubleArrayElements(env, frameTimeStamp, ts, 0);
	convertToRGBAWithStride(IMAGE_STRIDE_FISHEYE, IMAGE_WIDTH_FISHEYE, IMAGE_HEIGHT_FISHEYE, fisheye_image_buffer_copy, fisheyeCameraImageBufferRGBA);
	(*env)->SetByteArrayRegion (env, pixelBufferFisheye, 0, bufferSizeFisheye, fisheyeCameraImageBufferRGBA);
	pthread_mutex_unlock(&fisheyeImageLock);

	return pixelBufferFisheye;
}

JNIEXPORT jdouble JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_getColorFrameTimestamp(JNIEnv *env, jobject This)
{
	double lastColorFrameTimeStampCopy;

	pthread_mutex_lock(&colorImageLock);
	lastColorFrameTimeStampCopy = lastColorFrameTimeStamp;
	pthread_mutex_unlock(&colorImageLock);

	return lastColorFrameTimeStampCopy;
}


JNIEXPORT jdouble JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_getFisheyeFrameTimestamp(JNIEnv *env, jobject This)
{
	double lastFisheyeFrameTimeStampCopy;

	pthread_mutex_lock(&fisheyeImageLock);
	lastFisheyeFrameTimeStampCopy = lastFisheyeFrameTimeStamp;
	pthread_mutex_unlock(&fisheyeImageLock);

	return lastFisheyeFrameTimeStampCopy;
}

JNIEXPORT jdoubleArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnPoseArray(JNIEnv *env, jobject This)
{
	jdoubleArray poseDoubleArray = (*env)->NewDoubleArray(env, 10);
	pthread_mutex_lock(&poseLock);
	(*env)->SetDoubleArrayRegion (env, poseDoubleArray, 0, 10, poseBuffer);
	pthread_mutex_unlock(&poseLock);
    return poseDoubleArray;
}


JNIEXPORT jdoubleArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnPoseAreaArray(JNIEnv *env, jobject This)
{
	jdoubleArray poseDoubleArray = (*env)->NewDoubleArray(env, 10);
	pthread_mutex_lock(&areaPoseLock);
	(*env)->SetDoubleArrayRegion (env, poseDoubleArray, 0, 10, poseBufferArea);
	pthread_mutex_unlock(&areaPoseLock);
	return poseDoubleArray;
}

JNIEXPORT jdoubleArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnIntrinsicsColor(JNIEnv *env, jobject This)
{
	double intrinsicsArray[11];
	jdoubleArray intrinsicsDoubleArray = (*env)->NewDoubleArray(env, 11);
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

	(*env)->SetDoubleArrayRegion(env, intrinsicsDoubleArray, 0, 11, intrinsicsArray);
	return intrinsicsDoubleArray;
}

JNIEXPORT jdoubleArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnIntrinsicsFisheye(JNIEnv *env, jobject This)
{
	double intrinsicsArray[11];
	jdoubleArray intrinsicsDoubleArray = (*env)->NewDoubleArray(env, 11);
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

	(*env)->SetDoubleArrayRegion(env, intrinsicsDoubleArray, 0, 11, intrinsicsArray);
	return intrinsicsDoubleArray;
}

JNIEXPORT jfloatArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnPointCloud(JNIEnv *env, jobject This)
{
	// could have a race condition here
	pthread_mutex_lock(&pointCloudLock);
	int xyzValidCopy = xyzValid;
	jfloatArray pointCloudFloatArray = (*env)->NewFloatArray(env, xyzValidCopy*3+1);
	(*env)->SetFloatArrayRegion (env, pointCloudFloatArray, 0, xyzValidCopy*3+1, xyzBuffer);
	pthread_mutex_unlock(&pointCloudLock);

	return pointCloudFloatArray;
}
