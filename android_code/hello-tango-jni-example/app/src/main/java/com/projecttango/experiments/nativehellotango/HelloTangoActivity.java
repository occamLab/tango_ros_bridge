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

import com.projecttango.experiments.nativehellotango.TangoInitializationHelper;

import java.io.BufferedReader;
import java.nio.ByteBuffer;

import android.graphics.Bitmap;

import com.google.tango.hellotangojni.R;

import android.os.IBinder;
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

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.InetAddress;
import java.io.File;
import java.io.PrintWriter;
import java.io.ByteArrayOutputStream;
import java.net.Socket;
import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;

import android.content.SharedPreferences;
import android.content.ComponentName;
import android.content.ServiceConnection;

import java.util.Arrays;

/**
 * Main activity controls Tango lifecycle.
 */
public class HelloTangoActivity extends Activity {
    public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";

    private SharedPreferences preferences;
    private SharedPreferences.Editor preferencesEditor;

    private boolean connected = false;
    private String hostName = "";
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

    // Project Tango Service connection.
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            // Synchronization around HelloMotionTrackingActivity object is to avoid
            // Project Tango disconnect in the middle of the connecting operation.
            // Set binder object to C API through JNI
            // Call TangoService_setBinder(env, service);

            // At this point we could call Project Tango functions.
            // For example:
            TangoJNINative.setBinder(service);
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

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shut down/retry in the event
            // that Project Tango itself crashes/gets upgraded while running.
        }
    };


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
        TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
    }

    @Override
    protected void onPause() {
        super.onPause();
        // TODO: this behavior doesn't work properly (App will always crash on pause)
        System.out.println("MYDEBUG: DISCONNECTING FROM TANGO!!!");
        TangoJNINative.disconnect();
        synchronized (this) {
            unbindService(mTangoServiceConnection);
        }
    }
}
