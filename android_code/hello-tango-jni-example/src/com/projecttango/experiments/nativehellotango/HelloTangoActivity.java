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
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import android.content.SharedPreferences;
import java.util.Arrays;

/**
 * Main activity controls Tango lifecycle.
 */
public class HelloTangoActivity extends Activity {
  public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";
  public static final String EXTRA_VALUE_MOTION_TRACKING = "MOTION_TRACKING_PERMISSION";

  private SharedPreferences preferences;
  private SharedPreferences.Editor preferencesEditor;
  private boolean mIsPermissionIntentCalled = false;

  private boolean connected = false;
  private String hostName = ""; 
  final int portNumberImages = 11111;
  final int portNumberPointCloud = 11112;
  final int portNumberPose = 11113;
  final int portNumberIntrinsics = 11114;
  
  private Socket kkSocketImages;
  private PrintWriter outImages;
  
  private Socket kkSocketPose;
  private PrintWriter outPose;
  
  private Socket kkSocketPointCloud;
  private PrintWriter outPointCloud;
  
  private Socket kkSocketIntrinsics;
  private PrintWriter outIntrinsics;
  
  protected Bitmap bm; 
  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);
    setTitle(R.string.app_name);
	bm = Bitmap.createBitmap(1280, 720, Bitmap.Config.ARGB_8888);
    ImageView image = (ImageView) findViewById(R.id.test_image);
    image.setImageBitmap(bm);
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
					  kkSocketImages = new Socket(hostName, portNumberImages);
					  outImages = new PrintWriter(kkSocketImages.getOutputStream(), true);
	    	        
					  kkSocketPointCloud = new Socket(hostName, portNumberPointCloud);
					  outPointCloud = new PrintWriter(kkSocketPointCloud.getOutputStream(), true);
	    	        
					  kkSocketPose = new Socket(hostName, portNumberPose);
					  outPose = new PrintWriter(kkSocketPose.getOutputStream(), true);
					  
					  kkSocketIntrinsics = new Socket(hostName, portNumberIntrinsics);
					  outIntrinsics = new PrintWriter(kkSocketIntrinsics.getOutputStream(), true);

					  connected = true;
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
	  } else {
		  new AsyncTask<Void, Integer, Void>(){
			  @Override
			  protected Void doInBackground(Void... arg0) {
				  try {
					  kkSocketImages.close();
					  kkSocketPose.close();
					  kkSocketPointCloud.close();
					  kkSocketIntrinsics.close();
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
    if (requestCode == 0) {
        // Make sure the request was successful.
        if (resultCode == RESULT_CANCELED) {
          Toast.makeText(this, 
            "Motion Tracking Permission Needed!", Toast.LENGTH_SHORT).show();
          finish();
        } else {
          TangoJNINative.initialize(this);
          TangoJNINative.connectCallbacks();
          TangoJNINative.setupConfig();
          TangoJNINative.connect();
          mIsPermissionIntentCalled = true;

          new Thread(new Runnable() {
              public void run() { 
            	  while (true) {
            		  double[] currIntrinsics = TangoJNINative.returnIntrinsics();
            		  if (connected) {
            			  outIntrinsics.println("INTRINSICSSTARTINGRIGHTNOW");
            			  String intrinsicsAsString = Arrays.toString(currIntrinsics);
            			  outIntrinsics.println(intrinsicsAsString.substring(1,intrinsicsAsString.length() - 1));
	                	  outIntrinsics.println("INTRINSICSENDINGRIGHTNOW");
	            	  }
            		  try {
            			  Thread.sleep(1000);
            		  } catch (InterruptedException ex) {
            			  System.err.println("Something weird happened");
            		  }
	              }
              }
          }).start();
          
          new Thread(new Runnable() {
              public void run() { 
            	  while (true) {
            		  float[] currPointCloud = TangoJNINative.returnPointCloud();
            		  if (connected) {
            			  outPointCloud.println("POINTCLOUDSTARTINGRIGHTNOW");
            			  String pcAsString = Arrays.toString(currPointCloud);
            			  outPointCloud.println(pcAsString.substring(1,pcAsString.length() - 1));
	                	  outPointCloud.println("POINTCLOUDENDINGRIGHTNOW");
	            	  }
            		  try {
            			  Thread.sleep(1000);
            		  } catch (InterruptedException ex) {
            			  System.err.println("Something weird happened");
            		  }
	              }
              }
          }).start();
         
          new Thread(new Runnable() {
              public void run() {
            	  while (true) {
            		  double frameTimeStamp = TangoJNINative.getFrameTimestamp();
            		  byte[] myArray = TangoJNINative.returnArray();
            		  System.out.println(myArray.length);
            		  if (myArray != null && myArray.length != 0)  { 
        				  bm.copyPixelsFromBuffer(ByteBuffer.wrap(myArray));
        				  
                	      runOnUiThread(new Runnable() {
                	          @Override
                	          public void run() {
                	                ImageView image = (ImageView) findViewById(R.id.test_image);
                	                image.invalidate(); 
                	          }
                	       });
                	       if (connected) {
	                		   try {
	                			   outImages.println("DEPTHFRAMESTARTINGRIGHTNOW");
	                			   outImages.println("DEPTHTIMESTAMPSTARTINGRIGHTNOW");
	                			   outImages.println(frameTimeStamp);
	                			   outImages.println("DEPTHTIMESTAMPENDINGRIGHTNOW");
	                			   bm.compress(Bitmap.CompressFormat.JPEG, 50, kkSocketImages.getOutputStream());
	                			   outImages.println("DEPTHFRAMEENDINGRIGHTNOW");
	                		   } catch (IOException ex) {
	                			   ex.printStackTrace();
	                			   System.err.println("ERROR!"); 
	                		   }
                	       }
            		  }
            		  try {
            			  Thread.sleep(200);
            		  } catch (InterruptedException ex) {
            			  System.err.println("Something weird happened");
            		  }
            	  } 
              }
          }).start();
          new Thread(new Runnable() {
              public void run() {
            	  while (true) {
            		  double[] currPose = TangoJNINative.returnPoseArray();
            		  if (connected) {
            			  System.out.println("currPose.length  " + currPose.length);
	            		  outPose.println("POSESTARTINGRIGHTNOW");
	            		  for (int i = 0; i < currPose.length; i++) {
	            			  outPose.print(currPose[i]);
	            			  if (i + 1 < currPose.length) {
	            				  outPose.print(",");
	                		  }
	                	  }
	            		  outPose.println("POSEENDINGRIGHTNOW");
	            		  System.out.println("MYPOSE: " + Arrays.toString(currPose));
            		  }
            		  try {
            			  Thread.sleep(100);
            		  } catch (InterruptedException ex) {
            			  System.err.println("Something weird happened");
            		  }
            	  }
              }
          }).start();
        }
    }
  }
}
