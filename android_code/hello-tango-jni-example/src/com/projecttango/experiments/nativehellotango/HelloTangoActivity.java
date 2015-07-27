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

// TODO: handle disconnects from the socket server (reset everything)
// TODO: PointCloud2 instead of PointCloud (faster)
// TODO: Add a text box for the IP Address

package com.projecttango.experiments.nativehellotango;

import java.io.BufferedReader;
import java.nio.ByteBuffer;
 
import android.graphics.Bitmap;

import com.google.tango.hellotangojni.R;

import android.app.Activity;
import android.widget.ImageView;
import android.content.Intent;
import android.os.Bundle;
import android.os.AsyncTask;
import android.widget.ImageView;
import android.widget.Button;
import android.widget.EditText;
import android.view.View;
import android.widget.Toast;
import android.graphics.Color;
import android.os.Environment;

import java.io.File;
import java.io.PrintWriter;
import java.net.Socket;
import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;

import android.content.SharedPreferences;

import java.util.Arrays;

/**
 * Main activity controls Tango lifecycle.
 */
public class HelloTangoActivity extends Activity {
  public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";
  public static final String EXTRA_VALUE_MOTION_TRACKING = "MOTION_TRACKING_PERMISSION";
  public static final String EXTRA_VALUE_ADF = "ADF_LOAD_SAVE_PERMISSION";

  
  private SharedPreferences preferences;
  private SharedPreferences.Editor preferencesEditor;
  private boolean mIsPermissionIntentCalled = false;
 
  private boolean connected = false;
  private String hostName = ""; 
  final int portNumberColorImages = 11111;
  final int portNumberPointCloud = 11112;
  final int portNumberPose = 11113;
  final int portNumberIntrinsicsColor = 11114;
  final int portNumberFisheyeImages = 11115;
  final int portNumberIntrinsicsFisheye = 11116;
  final int portNumberPoseArea = 11117;
  
  private Socket kkSocketFisheyeImages;
  private PrintWriter outFisheyeImages;
  
  private Socket kkSocketColorImages;
  private PrintWriter outColorImages;
  
  private Socket kkSocketPose;
  private PrintWriter outPose;

  private Socket kkSocketPoseArea;
  private PrintWriter outPoseArea;
  
  private Socket kkSocketPointCloud;
  private PrintWriter outPointCloud;
  
  private Socket kkSocketIntrinsicsColor;
  private PrintWriter outIntrinsicsColor;
  
  private Socket kkSocketIntrinsicsFisheye;
  private PrintWriter outIntrinsicsFisheye;

  private Thread imagesColorThread;
  private Thread imagesFisheyeThread;
  private Thread poseThread;  
  private Thread poseAreaThread;

  
  private Thread pointCloudThread;
  private Thread intrinsicsColorThread;
  private Thread intrinsicsFisheyeThread;

  protected Bitmap bmColor;
  protected Bitmap bmFisheye; 

  @Override 
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);
    setTitle(R.string.app_name);
    // Tango fish eye
	bmFisheye = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
	bmColor = Bitmap.createBitmap(1280, 720, Bitmap.Config.ARGB_8888);

	// tango color camera
	//bm = Bitmap.createBitmap(1280, 720, Bitmap.Config.ARGB_8888);
    ImageView image = (ImageView) findViewById(R.id.test_image);
    image.setImageBitmap(bmColor);
    preferences = getSharedPreferences("myvals", 0);
    preferencesEditor = preferences.edit();
	EditText mEdit   = (EditText)findViewById(R.id.editText1);
	mEdit.setText(preferences.getString("ROS_HOST",""));
  } 
  
  public void toggleConnectionStatus(View v) {
	  if (!connected) {
		  EditText mEdit   = (EditText)findViewById(R.id.editText1);
		  hostName = mEdit.getText().toString();
		  System.out.println("button clicked!! " + mEdit.getText().toString());
		  preferencesEditor.putString("ROS_HOST",hostName);
		  preferencesEditor.commit(); 
		  new AsyncTask<Void, Integer, Void>(){
			  @Override 
			  protected Void doInBackground(Void... arg0) {
				  try  { 
					  kkSocketColorImages = new Socket(hostName, portNumberColorImages);
					  outColorImages = new PrintWriter(kkSocketColorImages.getOutputStream(), true);
					  
					  kkSocketFisheyeImages = new Socket(hostName, portNumberFisheyeImages);
					  System.out.println("CREATED SOCKET!");
					  outFisheyeImages = new PrintWriter(kkSocketFisheyeImages.getOutputStream(), true);
					  
					  kkSocketPointCloud = new Socket(hostName, portNumberPointCloud);
					  outPointCloud = new PrintWriter(kkSocketPointCloud.getOutputStream(), true);
	    	       
					  kkSocketPose = new Socket(hostName, portNumberPose);
					  outPose = new PrintWriter(kkSocketPose.getOutputStream(), true);
					  /*
					  kkSocketPoseArea = new Socket(hostName, portNumberPoseArea);
					  outPoseArea = new PrintWriter(kkSocketPoseArea.getOutputStream(), true);
					  */
					  kkSocketIntrinsicsColor = new Socket(hostName, portNumberIntrinsicsColor);
					  outIntrinsicsColor = new PrintWriter(kkSocketIntrinsicsColor.getOutputStream(), true);
					  
					  kkSocketIntrinsicsFisheye = new Socket(hostName, portNumberIntrinsicsFisheye);
					  outIntrinsicsFisheye = new PrintWriter(kkSocketIntrinsicsFisheye.getOutputStream(), true);

					  connected = true;
					  System.out.println("MYDEBUG: CONNECTED!");
				  } catch (Exception ex) {
					  System.out.println("MYDEBUG: " + ex);
					  ex.printStackTrace();
				  }
				  return null;
			  } 

			  @Override         
			  protected void onPostExecute(Void result) { 
				  if (connected) {
					  ((Button)findViewById(R.id.button1)).setText("Disconnect");
				  } else {
					  ((Button)findViewById(R.id.button1)).setText("Connect");
				  }
			  }
			  @Override        
			  protected void onPreExecute() { 
			  }
		  }.execute((Void)null);
	  } else {
		  new AsyncTask<Void, Integer, Void>(){
			  @Override
			  protected Void doInBackground(Void... arg0) {
				  try {
					  kkSocketColorImages.close();
					  kkSocketFisheyeImages.close();
					  kkSocketPose.close();
					 // kkSocketPoseArea.close();
					  kkSocketPointCloud.close();
					  kkSocketIntrinsicsColor.close();
					  kkSocketIntrinsicsFisheye.close();
					  connected = false;
					  System.out.println("CLOSINGDOWN!!!");
				  } catch (Exception ex) {
					  ex.printStackTrace();
				  }
				  return null;
			  }
			  @Override         
			  protected void onPostExecute(Void result) { 
				  if (connected) {
					  ((Button)findViewById(R.id.button1)).setText("Disconnect");
				  } else {
					  ((Button)findViewById(R.id.button1)).setText("Connect");
				  }
			  }
			  @Override        
			  protected void onPreExecute() { 
			  }
		  }.execute((Void)null);
	  }
  }
  
  @Override
  protected void onResume() {
    super.onResume(); 
    if (!mIsPermissionIntentCalled) {
      Intent intent = new Intent();
      intent.setAction("android.intent.action.REQUEST_TANGO_PERMISSION");
      intent.putExtra(EXTRA_KEY_PERMISSIONTYPE, EXTRA_VALUE_MOTION_TRACKING);
      startActivityForResult(intent, 0);

      // Invoke intent to give permission to use Area Description/Learning features.
      intent = new Intent();
      intent.setAction("android.intent.action.REQUEST_TANGO_PERMISSION");
      intent.putExtra(EXTRA_KEY_PERMISSIONTYPE, EXTRA_VALUE_ADF);
      startActivityForResult(intent, 1);
    }
  }

  @Override
  protected void onPause() {
    super.onPause();
    // TODO: this behavior doesn't work properly (App will always crash on pause)
    TangoJNINative.disconnect();
    mIsPermissionIntentCalled = false;
  } 

  @Override 
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    // Check which request we're responding to.
    if (requestCode == 1) {
        // Make sure the request was successful.
        if (resultCode == RESULT_CANCELED) {
          Toast.makeText(this, 
            "Motion Tracking Permission Needed!", Toast.LENGTH_SHORT).show();
          finish();
        } else {
          TangoJNINative.initialize(this);
          TangoJNINative.setupConfig();
          TangoJNINative.connectCallbacks();
          TangoJNINative.connect();
          mIsPermissionIntentCalled = true;
 
          intrinsicsFisheyeThread = new Thread(new Runnable() {
              public void run() {
            	  while (true) {
            		  double[] currIntrinsics = TangoJNINative.returnIntrinsicsFisheye();
            		  if (connected) {
            			  outIntrinsicsFisheye.println("INTRINSICSSTARTINGRIGHTNOW");
            			  String intrinsicsAsString = Arrays.toString(currIntrinsics);
            			  outIntrinsicsFisheye.println(intrinsicsAsString.substring(1,intrinsicsAsString.length() - 1));
	                	  outIntrinsicsFisheye.println("INTRINSICSENDINGRIGHTNOW");
	            	  }
            		  try {
            			  Thread.sleep(1000);
            		  } catch (InterruptedException ex) {
            			  System.err.println("Something weird happened");
            		  }
	              }
              }
          });
         
          intrinsicsColorThread = new Thread(new Runnable() {
              public void run() {
            	  while (true) {
            		  double[] currIntrinsics = TangoJNINative.returnIntrinsicsColor();
            		  if (connected) {
            			  outIntrinsicsColor.println("INTRINSICSSTARTINGRIGHTNOW");
            			  String intrinsicsAsString = Arrays.toString(currIntrinsics);
            			  outIntrinsicsColor.println(intrinsicsAsString.substring(1,intrinsicsAsString.length() - 1));
	                	  outIntrinsicsColor.println("INTRINSICSENDINGRIGHTNOW");
	            	  }
            		  try {
            			  Thread.sleep(1000);
            		  } catch (InterruptedException ex) {
            			  System.err.println("Something weird happened");
            		  }
	              }
              }
          });
         
          
          pointCloudThread = new Thread(new Runnable() {
              public void run() { 
            	  while (true) {
            		  float[] currPointCloud = TangoJNINative.returnPointCloud();
            		  imagesColorThread.interrupt();
            		  System.out.println("POINT CLOUD SIZE " + currPointCloud.length);
            		  if (connected) {
            			  outPointCloud.println("POINTCLOUDSTARTINGRIGHTNOW");
            			  try {
            				  DataOutputStream os = new DataOutputStream(kkSocketPointCloud.getOutputStream());
            				  byte buf[] = new byte[4*currPointCloud.length];
            				  for (int i=0; i<currPointCloud.length; ++i)
            				  {
            				      int val = Float.floatToRawIntBits(currPointCloud[i]);
            				      buf[4 * i] = (byte) (val >> 24);
            				      buf[4 * i + 1] = (byte) (val >> 16) ;
            				      buf[4 * i + 2] = (byte) (val >> 8);
            				      buf[4 * i + 3] = (byte) (val);
            				  } 
            				  os.write(buf);
            			  } catch (Exception ex) { 
            				  System.out.println("WEIRDNESS");
            			  } 
            			  System.out.println("pcAs WROTE TO SOCKET!");
            			  outPointCloud.println();
	                	  outPointCloud.println("POINTCLOUDENDINGRIGHTNOW");
	            	  }
            		  try {
            			  Thread.sleep(1000);
            		  } catch (InterruptedException ex) {
            			  System.err.println("Something weird happened");
            		  }
	              } 
              }
          });
        
          imagesColorThread = new Thread(new Runnable() {
              public void run() {
            	  while (true) { 
            		  double frameTimeStamp = TangoJNINative.getColorFrameTimestamp();
            		  byte[] myArray = TangoJNINative.returnArrayColor();
            		  if (myArray != null && myArray.length != 0)  { 
        				  bmColor.copyPixelsFromBuffer(ByteBuffer.wrap(myArray));

                	       if (connected) {
	                		   try {
	                			   outColorImages.println("DEPTHFRAMESTARTINGRIGHTNOW");
	                			   outColorImages.println("DEPTHTIMESTAMPSTARTINGRIGHTNOW");
	                			   outColorImages.println(frameTimeStamp);
	                			   outColorImages.println("DEPTHTIMESTAMPENDINGRIGHTNOW");
	                			   bmColor.compress(Bitmap.CompressFormat.JPEG, 50, kkSocketColorImages.getOutputStream());
	                			   outColorImages.println("DEPTHFRAMEENDINGRIGHTNOW");
	                		   } catch (IOException ex) {
	                			   ex.printStackTrace();
	                			   System.err.println("ERROR!"); 
	                		   }
                	       }
            		  }
            	      runOnUiThread(new Runnable() {
            	          @Override
            	          public void run() {
            	                ImageView image = (ImageView) findViewById(R.id.test_image);
            	                image.invalidate(); 
            	          }
            	       });
            		  try {
            			  // this is pretty damn fast... would be nice to have some flow control
            			  Thread.sleep(30);
            		  } catch (InterruptedException ex) {
            			 // System.err.println("Something weird happened");
            		  }
            	  } 
              }
          });
           
          imagesFisheyeThread = new Thread(new Runnable() {
              public void run() {
            	  while (true) {
            		  double frameTimeStamp = TangoJNINative.getFisheyeFrameTimestamp();
            		  System.out.println("return array fisheye");
            		  byte[] myArray = TangoJNINative.returnArrayFisheye();
            		  System.out.println("Done returning array fisheye");
            		  if (myArray != null && myArray.length != 0)  { 
        				  bmFisheye.copyPixelsFromBuffer(ByteBuffer.wrap(myArray));
 
                	       if (connected) { 
	                		   try {
	                			   outFisheyeImages.println("DEPTHFRAMESTARTINGRIGHTNOW");
	                			   outFisheyeImages.println("DEPTHTIMESTAMPSTARTINGRIGHTNOW");
	                			   outFisheyeImages.println(frameTimeStamp);
	                			   outFisheyeImages.println("DEPTHTIMESTAMPENDINGRIGHTNOW");
	                			   bmFisheye.compress(Bitmap.CompressFormat.JPEG, 50, kkSocketFisheyeImages.getOutputStream());
	                			   outFisheyeImages.println("DEPTHFRAMEENDINGRIGHTNOW");
	                		   } catch (IOException ex) {
	                			   ex.printStackTrace();
	                			   System.err.println("ERROR!"); 
	                		   }
                	       }
            		  }
            		  try {
            			  // this is pretty damn fast... would be nice to have some flow control
            			  Thread.sleep(30);
            		  } catch (InterruptedException ex) {
            			 // System.err.println("Something weird happened");
            		  }
            	  } 
              }
          });
          poseThread = new Thread(new Runnable() {
              public void run() { 
            	  while (true) {
            		  double[] currPose = TangoJNINative.returnPoseArray();
            		  if (connected) { 
	            		  outPose.println("POSESTARTINGRIGHTNOW");
	            		  for (int i = 0; i < currPose.length; i++) {
	            			  outPose.print(currPose[i]);
	            			  if (i + 1 < currPose.length) {
	            				  outPose.print(",");
	                		  }
	                	  }
	            		  outPose.println("POSEENDINGRIGHTNOW");
            		  }
            		  try { 
            			  Thread.sleep(100);
            		  } catch (InterruptedException ex) {
            			  System.err.println("Something weird happened");
            		  }
            	  }
              }
          });
/*
          poseAreaThread = new Thread(new Runnable() {
              public void run() { 
            	  while (true) {
            		  double[] currPose = TangoJNINative.returnPoseAreaArray();
            		  if (connected) { 
	            		  outPoseArea.println("POSESTARTINGRIGHTNOW");
	            		  for (int i = 0; i < currPose.length; i++) {
	            			  outPoseArea.print(currPose[i]);
	            			  if (i + 1 < currPose.length) {
	            				  outPoseArea.print(",");
	                		  }
	                	  }
	            		  outPoseArea.println("POSEENDINGRIGHTNOW");
            		  } 
            		  try { 
            			  Thread.sleep(100);
            		  } catch (InterruptedException ex) {
            			  System.err.println("Something weird happened");
            		  }
            	  }
              }
          });
          */
          poseThread.start();
          //poseAreaThread.start();
          pointCloudThread.start();
          intrinsicsColorThread.start();
          intrinsicsFisheyeThread.start();
          imagesColorThread.start();
          imagesFisheyeThread.start();

        }
    }
  }
}
