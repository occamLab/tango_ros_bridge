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

package com.projecttango.examples.cpp.rgbdepthsync;
import android.os.IBinder;
import android.util.Log;

import com.projecttango.examples.cpp.util.TangoInitializationHelper;

/**
 * Interfaces between C and Java.
 */
public class TangoJNINative {
  static {
    // This project depends on tango_client_api, so we need to make sure we load
    // the correct library first.
    if (TangoInitializationHelper.loadTangoSharedLibrary() ==
            TangoInitializationHelper.ARCH_ERROR) {
      Log.e("TangoJNINative", "ERROR! Unable to load libtango_client_api.so!");
    }
    System.loadLibrary("cpp_rgb_depth_sync_example");
  }

  public static native void setBinder(IBinder service);

  public static native void setupConfig(boolean streamColorImages);

  public static native void connectCallbacks(boolean streamColorImages);

  public static native void connect();

  public static native void disconnect();
  // byte[] instead of void
  public static native byte[] returnArrayColor(double[] frameTimestamp, int[] dimensions);

  public static native byte[] returnArrayFisheye(double[] frameTimestamp, int[] stride);

  public static native double getColorFrameTimestamp();

  public static native double getFisheyeFrameTimestamp();

  public static native double[] getPoses(double[] poseTimestamps);

  public static native double[] returnPoseArray();

  public static native double[] returnPoseAreaArray();

  public static native float[] returnPointCloud();

  public static native double[] returnIntrinsicsColor();

  public static native double[] returnIntrinsicsFisheye();
}