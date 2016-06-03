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

package com.projecttango.experiments.nativehellotango;

import android.app.Activity;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.os.AsyncTask;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Toast;

import com.google.tango.hellotangojni.R;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.ByteArrayOutputStream;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketException;
import java.nio.ByteBuffer;
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

    // keeps track of whether we have requested permissions for Area learning and Motion Tracking
    private boolean mIsPermissionIntentCalled = false;

    // keeps track of whether or not the sockets are connected
    private boolean connected = false;
    // the hostname (or IP address) to stream the data to
    private String hostName = "";

    // These are the UDP ports that will be used to broadcast the sensor data
    final int portNumberColorImages = 11111;
    final int portNumberPointCloud = 11112;
    final int portNumberPose = 11113;
    final int portNumberIntrinsicsColor = 11114;
    final int portNumberFisheyeImages = 11115;
    final int portNumberIntrinsicsFisheye = 11116;
    final int portNumberPoseArea = 11117;

    private DatagramSocket udpFisheyeImages;
    private DatagramSocket udpColorImages;
    private DatagramSocket udpPointCloud;
    private DatagramSocket udpPose;
    private DatagramSocket udpAreaPose;
    private DatagramSocket udpIntrinsicsColor;
    private DatagramSocket udpIntrinsicsFisheye;

    private Thread imagesColorThread;
    private Thread imagesFisheyeThread;
    private Thread poseThread;
    private Thread poseAreaThread;


    private Thread pointCloudThread;
    private Thread intrinsicsColorThread;
    private Thread intrinsicsFisheyeThread;

    protected Bitmap bmColor;
    protected Bitmap bmFisheye;

    private InetAddress remote;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setTitle(R.string.app_name);
        // Tango fish eye
        bmFisheye = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
        // tango color camera
        bmColor = Bitmap.createBitmap(1280, 720, Bitmap.Config.ARGB_8888);

        ImageView image = (ImageView) findViewById(R.id.test_image);
        image.setImageBitmap(bmColor);
        preferences = getSharedPreferences("myvals", 0);
        preferencesEditor = preferences.edit();
        EditText mEdit = (EditText) findViewById(R.id.editText1);
        mEdit.setText(preferences.getString("ROS_HOST", ""));
    }

    public void toggleConnectionStatus(View v) {
        if (!connected) {
            EditText mEdit = (EditText) findViewById(R.id.editText1);
            hostName = mEdit.getText().toString();
            try {
                remote = InetAddress.getByName(hostName);
            } catch (Exception ex) {
                ex.printStackTrace();
            }
            System.out.println("button clicked!! " + mEdit.getText().toString());
            preferencesEditor.putString("ROS_HOST", hostName);
            preferencesEditor.commit();
            new AsyncTask<Void, Integer, Void>() {
                @Override
                protected Void doInBackground(Void... arg0) {
                    try {
                        // since we are using UDP there isn't much we have to do, just start
                        // blasting out the data
                        connected = true;
                    } catch (Exception ex) {
                        ex.printStackTrace();
                    }
                    return null;
                }

                @Override
                protected void onPostExecute(Void result) {
                    if (connected) {
                        ((Button) findViewById(R.id.button1)).setText("Disconnect");
                    } else {
                        ((Button) findViewById(R.id.button1)).setText("Connect");
                    }
                }

                @Override
                protected void onPreExecute() {
                }
            }.execute((Void) null);
        } else {
            new AsyncTask<Void, Integer, Void>() {
                @Override
                protected Void doInBackground(Void... arg0) {
                    try {
                        // nothing much to do, everything is UDP, just stop sending data
                        connected = false;
                    } catch (Exception ex) {
                        ex.printStackTrace();
                    }
                    return null;
                }

                @Override
                protected void onPostExecute(Void result) {
                    if (connected) {
                        ((Button) findViewById(R.id.button1)).setText("Disconnect");
                    } else {
                        ((Button) findViewById(R.id.button1)).setText("Connect");
                    }
                }

                @Override
                protected void onPreExecute() {
                }
            }.execute((Void) null);
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!mIsPermissionIntentCalled) {
            // Invoke intent to give persmission to use Motion Tracking
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
                // mark the fact that we've successfully gotten permission to use area learning
                // and motion tracking
                mIsPermissionIntentCalled = true;

                // initialize, configure, and connect to the Tango Service using JNI calls
                TangoJNINative.initialize(this);
                TangoJNINative.setupConfig();
                TangoJNINative.connectCallbacks();
                TangoJNINative.connect();

                try {
                    udpFisheyeImages = new DatagramSocket();
                    udpColorImages = new DatagramSocket();
                    udpPointCloud = new DatagramSocket();
                    udpPose = new DatagramSocket();
                    udpAreaPose = new DatagramSocket();
                    udpIntrinsicsColor = new DatagramSocket();
                    udpIntrinsicsFisheye = new DatagramSocket();
                } catch (SocketException e) {
                    e.printStackTrace();
                }
                intrinsicsFisheyeThread = new Thread(new Runnable() {
                    public void run() {
                        while (true) {
                            double[] currIntrinsics = TangoJNINative.returnIntrinsicsFisheye();
                            if (connected) {
                                ByteArrayOutputStream udpPayload = new ByteArrayOutputStream();
                                String intrinsicsAsString = Arrays.toString(currIntrinsics);

                                try {
                                    udpPayload.write("INTRINSICSSTARTINGRIGHTNOW\n".getBytes());
                                    udpPayload.write((intrinsicsAsString.substring(1, intrinsicsAsString.length() - 1) + "\n").getBytes());
                                    udpPayload.write("INTRINSICSENDINGRIGHTNOW\n".getBytes());
                                    byte[] payload = udpPayload.toByteArray();
                                    udpIntrinsicsFisheye.send(new DatagramPacket(payload, payload.length, remote, portNumberIntrinsicsFisheye));
                                } catch (IOException e) {
                                    e.printStackTrace();
                                }
                            }
                            try {
                                // throttle the speed at which we send data
                                // TODO: allow this to be configured via some sort of user interface widget
                                Thread.sleep(100);
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
                                ByteArrayOutputStream udpPayload = new ByteArrayOutputStream();
                                String intrinsicsAsString = Arrays.toString(currIntrinsics);

                                try {
                                    udpPayload.write("INTRINSICSSTARTINGRIGHTNOW\n".getBytes());
                                    udpPayload.write((intrinsicsAsString.substring(1, intrinsicsAsString.length() - 1) + "\n").getBytes());
                                    udpPayload.write("INTRINSICSENDINGRIGHTNOW\n".getBytes());
                                    byte[] payload = udpPayload.toByteArray();
                                    udpIntrinsicsColor.send(new DatagramPacket(payload, payload.length, remote, portNumberIntrinsicsColor));
                                } catch (IOException e) {
                                    e.printStackTrace();
                                }
                            }
                            try {
                                // throttle the speed at which we send data
                                // TODO: allow this to be configured via some sort of user interface widget
                                Thread.sleep(100);
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
                            //imagesColorThread.interrupt();
                            System.out.println("POINT CLOUD SIZE " + currPointCloud.length);
                            if (connected) {
                                ByteArrayOutputStream udpPayload = new ByteArrayOutputStream();
                                try {
                                    udpPayload.write("POINTCLOUDSTARTINGRIGHTNOW\n".getBytes());
                                    // was throttling this before
                                    int currPointCloudLength = currPointCloud.length > 15000 ? 15000 : currPointCloud.length;
                                    byte buf[] = new byte[4 * currPointCloudLength];

                                    for (int i = 0; i < currPointCloudLength; ++i) {
                                        int val = Float.floatToRawIntBits(currPointCloud[i]);
                                        buf[4 * i] = (byte) (val >> 24);
                                        buf[4 * i + 1] = (byte) (val >> 16);
                                        buf[4 * i + 2] = (byte) (val >> 8);
                                        buf[4 * i + 3] = (byte) (val);
                                    }
                                    udpPayload.write(buf);
                                    udpPayload.write("\n".getBytes());
                                    udpPayload.write("POINTCLOUDENDINGRIGHTNOW\n".getBytes());
                                    byte[] payload = udpPayload.toByteArray();
                                    udpPointCloud.send(new DatagramPacket(payload, payload.length, remote, portNumberPointCloud));
                                } catch (Exception ex) {
                                    System.out.println("ERROR:  " + ex.getMessage());
                                }
                            }
                            try {
                                // throttle the speed at which we send data
                                // TODO: allow this to be configured via some sort of user interface widget
                                Thread.sleep(500);
                            } catch (InterruptedException ex) {
                                System.err.println("Thread was interrupted!");
                            }
                        }
                    }
                });

                imagesColorThread = new Thread(new Runnable() {
                    public void run() {
                        while (true) {
                            double frameTimeStamp = TangoJNINative.getColorFrameTimestamp();
                            byte[] myArray = TangoJNINative.returnArrayColor();
                            if (myArray != null && myArray.length != 0) {
                                bmColor.copyPixelsFromBuffer(ByteBuffer.wrap(myArray));

                                if (connected) {
                                    try {
                                        ByteArrayOutputStream udpPayload = new ByteArrayOutputStream();
                                        udpPayload.write("DEPTHFRAMESTARTINGRIGHTNOW\n".getBytes());
                                        udpPayload.write("DEPTHTIMESTAMPSTARTINGRIGHTNOW\n".getBytes());
                                        String frameTimeStampAsString = String.valueOf(frameTimeStamp);
                                        udpPayload.write((frameTimeStampAsString + "\n").getBytes());
                                        udpPayload.write("DEPTHTIMESTAMPENDINGRIGHTNOW\n".getBytes());
                                        bmColor.compress(Bitmap.CompressFormat.JPEG, 50, udpPayload);
                                        udpPayload.write("DEPTHFRAMEENDINGRIGHTNOW\n".getBytes());
                                        byte[] payload = udpPayload.toByteArray();
                                        udpColorImages.send(new DatagramPacket(payload, payload.length, remote, portNumberColorImages));
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
                                // throttle the speed at which we send data
                                // TODO: allow this to be configured via some sort of user interface widget
                                Thread.sleep(100);
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
                            byte[] myArray = TangoJNINative.returnArrayFisheye();
                            if (myArray != null && myArray.length != 0) {
                                bmFisheye.copyPixelsFromBuffer(ByteBuffer.wrap(myArray));

                                if (connected) {
                                    try {
                                        ByteArrayOutputStream udpPayload = new ByteArrayOutputStream();
                                        udpPayload.write("DEPTHFRAMESTARTINGRIGHTNOW\n".getBytes());
                                        udpPayload.write("DEPTHTIMESTAMPSTARTINGRIGHTNOW\n".getBytes());
                                        String frameTimeStampAsString = String.valueOf(frameTimeStamp);
                                        udpPayload.write((frameTimeStampAsString + "\n").getBytes());
                                        udpPayload.write("DEPTHTIMESTAMPENDINGRIGHTNOW\n".getBytes());
                                        bmFisheye.compress(Bitmap.CompressFormat.JPEG, 50, udpPayload);
                                        udpPayload.write("DEPTHFRAMEENDINGRIGHTNOW\n".getBytes());
                                        byte[] payload = udpPayload.toByteArray();
                                        udpFisheyeImages.send(new DatagramPacket(payload, payload.length, remote, portNumberFisheyeImages));
                                    } catch (IOException ex) {
                                        ex.printStackTrace();
                                        System.err.println("ERROR!");
                                    }
                                }
                            }
                            try {
                                // throttle the speed at which we send data
                                // TODO: allow this to be configured via some sort of user interface widget
                                Thread.sleep(100);
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
                                ByteArrayOutputStream udpPayload = new ByteArrayOutputStream();
                                try {
                                    udpPayload.write("POSESTARTINGRIGHTNOW\n".getBytes());
                                    for (int i = 0; i < currPose.length; i++) {
                                        udpPayload.write(String.valueOf(currPose[i]).getBytes());

                                        if (i + 1 < currPose.length) {
                                            udpPayload.write(",".getBytes());
                                        }
                                    }
                                    udpPayload.write("POSEENDINGRIGHTNOW\n".getBytes());
                                    byte[] payload = udpPayload.toByteArray();
                                    udpPose.send(new DatagramPacket(payload, payload.length, remote, portNumberPose));
                                } catch (IOException e) {
                                    e.printStackTrace();
                                }
                            }
                            try {
                                // throttle the speed at which we send data
                                // TODO: allow this to be configured via some sort of user interface widget
                                Thread.sleep(100);
                            } catch (InterruptedException ex) {
                                System.err.println("Something weird happened");
                            }
                        }
                    }
                });

                poseAreaThread = new Thread(new Runnable() {
                    public void run() {
                        while (true) {
                            double[] currPose = TangoJNINative.returnPoseAreaArray();
                            if (connected) {
                                ByteArrayOutputStream udpPayload = new ByteArrayOutputStream();
                                try {
                                    udpPayload.write("POSESTARTINGRIGHTNOW\n".getBytes());
                                    for (int i = 0; i < currPose.length; i++) {
                                        udpPayload.write(String.valueOf(currPose[i]).getBytes());

                                        if (i + 1 < currPose.length) {
                                            udpPayload.write(",".getBytes());
                                        }
                                    }
                                    udpPayload.write("POSEENDINGRIGHTNOW\n".getBytes());
                                    byte[] payload = udpPayload.toByteArray();
                                    udpAreaPose.send(new DatagramPacket(payload, payload.length, remote, portNumberPoseArea));
                                } catch (IOException e) {
                                    e.printStackTrace();
                                }
                            }
                            try {
                                // throttle the speed at which we send data
                                // TODO: allow this to be configured via some sort of user interface widget
                                Thread.sleep(100);
                            } catch (InterruptedException ex) {
                                System.err.println("Something weird happened");
                            }
                        }
                    }
                });
                // start all of the threads
                // TODO: allow these to be turned on and off from the app
                poseThread.start();
                poseAreaThread.start();
                pointCloudThread.start();
                intrinsicsColorThread.start();
                intrinsicsFisheyeThread.start();
                imagesColorThread.start();
                imagesFisheyeThread.start();
            }
        }
    }
}