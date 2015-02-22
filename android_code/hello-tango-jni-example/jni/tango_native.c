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
double poseBuffer[7];


unsigned char
clamp(float v, unsigned char min, unsigned char max) {
	if (v > max) {
		return max;
	}
	if (v < min) {
		return min;
	}
	return (unsigned char) ((int)v);
}

void yuv2rgb(float yValue, float uValue, float vValue,
        unsigned char *r, unsigned char *g, unsigned char *b) {
    float r_as_float = yValue + (1.370705 * (vValue-128));
    float g_as_float = yValue - (0.698001 * (vValue-128)) - (0.337633 * (uValue-128));
    float b_as_float = yValue + (1.732446 * (uValue-128));

    *r = clamp(r_as_float, 0, 255);
    *g = clamp(g_as_float, 0, 255);
    *b = clamp(b_as_float, 0, 255);
}

static void onFrameAvailable(void* context, TangoCameraId camera, const TangoImageBuffer* buffer) {
	memcpy(cameraImageBufferRGBA, buffer->data, bufferSize);
}

static void onXYZijAvailable(void* context, const TangoXYZij* XYZ_ij) {
	// clear out any old points
	int i;
	for (i = 0; i < 3*20000; i++) {
		xyzBuffer[i] = 0;
	}
	for (i = 0; i < XYZ_ij->xyz_count; i++) {
		xyzBuffer[i*3] = XYZ_ij->xyz[i][0];
		xyzBuffer[i*3+1] = XYZ_ij->xyz[i][1];
		xyzBuffer[i*3+2] = XYZ_ij->xyz[i][2];
	}
	xyzValid = XYZ_ij->xyz_count;
	LOGI("connectOnXYZijAvailable Number of ij points: %i %i\n", XYZ_ij->ij_rows, XYZ_ij->ij_cols);
	LOGI("connectOnXYZijAvailable Number of points: %i\n", XYZ_ij->xyz_count);
}

static void onPoseAvailable(void* context, const TangoPoseData* pose) {
 /* LOGI("Position: %f, %f, %f. Orientation: %f, %f, %f, %f",
       pose->translation[0], pose->translation[1], pose->translation[2],
       pose->orientation[0], pose->orientation[2], pose->orientation[3],
       pose->orientation[3]);*/
	poseBuffer[0] = pose->translation[0];
	poseBuffer[1] = pose->translation[1];
	poseBuffer[2] = pose->translation[2];
	poseBuffer[3] = pose->orientation[0];
	poseBuffer[4] = pose->orientation[1];
	poseBuffer[5] = pose->orientation[2];
	poseBuffer[6] = pose->orientation[3];
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

  TangoCoordinateFramePair pair = {TANGO_COORDINATE_FRAME_START_OF_SERVICE, TANGO_COORDINATE_FRAME_DEVICE };
  if (TangoService_connectOnPoseAvailable(1, &pair, onPoseAvailable)
      != TANGO_SUCCESS) {
    LOGI("TangoService_connectOnPoseAvailable(): Failed");
    return false;
  }
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

//jbyteArray instead of void
JNIEXPORT jbyteArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnArray(JNIEnv *env, jobject This)
{
	pixelBuffer = (*env)->NewByteArray(env, bufferSize);
	(*env)->SetByteArrayRegion (env, pixelBuffer, 0, bufferSize, cameraImageBufferRGBA);
    return pixelBuffer;
}

JNIEXPORT jdoubleArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnPoseArray(JNIEnv *env, jobject This)
{
	jdoubleArray poseDoubleArray = (*env)->NewDoubleArray(env, 7);
	(*env)->SetDoubleArrayRegion (env, poseDoubleArray, 0, 7, poseBuffer);
    return poseDoubleArray;
}

JNIEXPORT jfloatArray JNICALL Java_com_projecttango_experiments_nativehellotango_TangoJNINative_returnPointCloud(JNIEnv *env, jobject This)
{
	// could have a race condition here
	int xyzValidCopy = xyzValid;
	jfloatArray pointCloudFloatArray = (*env)->NewFloatArray(env, xyzValidCopy*3);
	(*env)->SetFloatArrayRegion (env, pointCloudFloatArray, 0, xyzValidCopy*3, xyzBuffer);
    return pointCloudFloatArray;
}
