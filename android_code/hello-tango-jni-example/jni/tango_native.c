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
#include <tango_client_api.h>
#include <stdio.h>

#define LOG_TAG "hello-tango-jni"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

// Tango configuration file.
// Configuration file describes current states of
// Tango Service.
TangoConfig config;

jbyteArray pixelBuffer = 0;

// TODO make these  class
unsigned int bufferSize = 4*1280*720;		// no need to hard code
unsigned char cameraImageBufferRGBA[4*1280*720];

float xyzBuffer[3*60000];			// assume 60000 is the highest number of points
int xyzValid = 0;
double poseBuffer[8];
double depthCameraPoseBuffer[8];
double lastFrameTimeStamp = 0.0;

static void onFrameAvailable(void* context, TangoCameraId camera, const TangoImageBuffer* buffer) {
	lastFrameTimeStamp = buffer->timestamp;
	memcpy(cameraImageBufferRGBA, buffer->data, bufferSize);
}

static void onXYZijAvailable(void* context, const TangoXYZij* XYZ_ij) {
	// clear out any old points
	int i;
	xyzValid = XYZ_ij->xyz_count;
	for (i = 0; i < 3*20000; i++) {
		xyzBuffer[i] = 0;
	}
	xyzBuffer[0] = XYZ_ij->timestamp;
	for (i = 1; i < XYZ_ij->xyz_count; i++) {
		xyzBuffer[(i-1)*3+1] = XYZ_ij->xyz[i][0];
		xyzBuffer[(i-1)*3+2] = XYZ_ij->xyz[i][1];
		xyzBuffer[(i-1)*3+3] = XYZ_ij->xyz[i][2];
	}
}

static void onPoseAvailable(void* context, const TangoPoseData* pose) {
	if (pose->frame.target == TANGO_COORDINATE_FRAME_DEVICE) {
		LOGI("Device Position: %f, %f, %f. Orientation: %f, %f, %f, %f",
		       pose->translation[0], pose->translation[1], pose->translation[2],
		       pose->orientation[0], pose->orientation[2], pose->orientation[3],
		       pose->orientation[3]);
		poseBuffer[0] = pose->translation[0];
		poseBuffer[1] = pose->translation[1];
		poseBuffer[2] = pose->translation[2];
		poseBuffer[3] = pose->orientation[0];
		poseBuffer[4] = pose->orientation[1];
		poseBuffer[5] = pose->orientation[2];
		poseBuffer[6] = pose->orientation[3];
		poseBuffer[7] = pose->timestamp; // the last is the time
	} else if (pose->frame.target == TANGO_COORDINATE_FRAME_CAMERA_COLOR) {
		LOGI("Depth Position: %f, %f, %f. Orientation: %f, %f, %f, %f",
		       pose->translation[0], pose->translation[1], pose->translation[2],
		       pose->orientation[0], pose->orientation[2], pose->orientation[3],
		       pose->orientation[3]);
		depthCameraPoseBuffer[0] = pose->translation[0];
		depthCameraPoseBuffer[1] = pose->translation[1];
		depthCameraPoseBuffer[2] = pose->translation[2];
		depthCameraPoseBuffer[3] = pose->orientation[0];
		depthCameraPoseBuffer[4] = pose->orientation[1];
		depthCameraPoseBuffer[5] = pose->orientation[2];
		depthCameraPoseBuffer[6] = pose->orientation[3];
		depthCameraPoseBuffer[7] = pose->timestamp;
	} else {
		LOGI("Unknown Frame Position: %f, %f, %f. Orientation: %f, %f, %f, %f",
		       pose->translation[0], pose->translation[1], pose->translation[2],
		       pose->orientation[0], pose->orientation[2], pose->orientation[3],
		       pose->orientation[3]);
	}
}

bool TangoInitialize(JNIEnv* env, jobject activity) {
  // Initialize Tango Service.
  // TODO(jguo): pass in env and jobject from activity.
  if (TangoService_initialize(env, activity) != TANGO_SUCCESS) {
    LOGE("TangoService_initialize(): Failed");
    return false;
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
  if (TangoConfig_setBool(config, "config_enable_depth", true) !=
      TANGO_SUCCESS) {
    LOGE("config_enable_depth Failed");
    return false;
  }
  return true;
}

bool TangoConnectCallbacks() {
  // Set listening pairs. Connect pose callback.
  // Note: the callback function should be re-connected 
  // after the application resumed from background.


  if (TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, 0, onFrameAvailable) != TANGO_SUCCESS) {
	  LOGI("frame available failed");
	  return false;
  }
	// Attach the onXYZijAvailable callback.
  if (TangoService_connectOnXYZijAvailable(onXYZijAvailable) != TANGO_SUCCESS) {
	  LOGI("TangoService_connectOnXYZijAvailable(): Failed");
	  return false;
  }

  TangoCoordinateFramePair* pairs = (TangoCoordinateFramePair*)malloc(2*sizeof(TangoCoordinateFramePair));
//  TangoCoordinateFramePair pair = {TANGO_COORDINATE_FRAME_START_OF_SERVICE, TANGO_COORDINATE_FRAME_DEVICE };
  pairs[0].base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pairs[0].target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  pairs[1].base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pairs[1].target = TANGO_COORDINATE_FRAME_DEVICE;
  // Currently cannot get the pose of the depth camera
  if (TangoService_connectOnPoseAvailable(2, pairs, onPoseAvailable)
      != TANGO_SUCCESS) {
    LOGI("TangoService_connectOnPoseAvailable(): Failed");
    return false;
  }
/*
  TangoCoordinateFramePair pair2 = {TANGO_COORDINATE_FRAME_START_OF_SERVICE, TANGO_COORDINATE_FRAME_CAMERA_DEPTH };
  if (TangoService_connectOnPoseAvailable(1, &pair2, onPoseAvailable)
      != TANGO_SUCCESS) {
    LOGI("TangoService_connectOnPoseAvailable(): Failed");
    return false;
  }
*/
  return true;
}

bool TangoConnect() {
  // Connect to the Tango Service.
  // Note: connecting Tango service will start the motion
  // tracking automatically.
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

JNIEXPORT void JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_initialize(JNIEnv* env, jobject obj, jobject activity)
{
  TangoInitialize(env, activity);
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

JNIEXPORT jbyteArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnArray(JNIEnv *env, jobject This)
{
	pixelBuffer = (*env)->NewByteArray(env, bufferSize);
	(*env)->SetByteArrayRegion (env, pixelBuffer, 0, bufferSize, cameraImageBufferRGBA);
    return pixelBuffer;
}

JNIEXPORT jdouble JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_getFrameTimestamp(JNIEnv *env, jobject This)
{
	return lastFrameTimeStamp;
}

JNIEXPORT jdoubleArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnPoseArray(JNIEnv *env, jobject This)
{
	jdoubleArray poseDoubleArray = (*env)->NewDoubleArray(env, 8);
	(*env)->SetDoubleArrayRegion (env, poseDoubleArray, 0, 8, poseBuffer);
    return poseDoubleArray;
}

JNIEXPORT jdoubleArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnIntrinsics(JNIEnv *env, jobject This)
{
	double intrinsicsArray[11];
	jdoubleArray intrinsicsDoubleArray = (*env)->NewDoubleArray(env, 11);
	TangoCameraIntrinsics ccIntrinsics;
	TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &ccIntrinsics);

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

JNIEXPORT jdoubleArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnDepthCameraPose(JNIEnv *env, jobject This)
{
	jdoubleArray poseDoubleArray = (*env)->NewDoubleArray(env, 8);
	(*env)->SetDoubleArrayRegion (env, poseDoubleArray, 0, 8, depthCameraPoseBuffer);
    return poseDoubleArray;
}

JNIEXPORT jfloatArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnPointCloud(JNIEnv *env, jobject This)
{
	// could have a race condition here
	int xyzValidCopy = xyzValid;
	jfloatArray pointCloudFloatArray = (*env)->NewFloatArray(env, xyzValidCopy*3+1);
	(*env)->SetFloatArrayRegion (env, pointCloudFloatArray, 0, xyzValidCopy*3+1, xyzBuffer);
    return pointCloudFloatArray;
}
