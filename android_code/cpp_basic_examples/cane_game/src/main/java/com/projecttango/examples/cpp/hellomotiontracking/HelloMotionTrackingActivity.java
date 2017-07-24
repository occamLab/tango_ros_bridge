/*
 * Copyright 2017 Paul Ruvolo. All Rights Reserved.
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
package com.projecttango.examples.cpp.hellomotiontracking;

import com.projecttango.examples.cpp.util.TangoInitializationHelper;

import android.graphics.YuvImage;
import android.os.IBinder;
import android.app.Activity;
import android.os.Bundle;
import java.io.ByteArrayOutputStream;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Rect;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Color;
import android.graphics.Matrix;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Spinner;
import android.widget.ArrayAdapter;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.content.ComponentName;
import android.content.ServiceConnection;

/**
 * Main activity controls Tango lifecycle.
 */
public class HelloMotionTrackingActivity extends Activity implements OnItemSelectedListener{
    public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";

    private boolean threadsStarted = false;

    private int globalSlot = 0;
    private double lastDisplayedImageTS = 0.0;

    private static int threadCount = 4;

    private Object fisheyeImageLock = new Object();
    private Object updateImageViewLock = new Object();

    // hardcoded for now :(
    private static int fisheyeImageWidth = 640;
    private static int fisheyeImageHeight = 480;

    private double targetFrameRate = 10.0;
    private double startingTimeStamp = -1.0;
    private int framesProcessed = 0;

    private Thread[] imagesFisheyeThread = new Thread[threadCount];

    public static Bitmap rotateBitmap(Bitmap source, float angle) {
        Matrix matrix = new Matrix();
        matrix.postRotate(angle);
        return Bitmap.createBitmap(source, 0, 0, source.getWidth(), source.getHeight(), matrix, true);
    }

    public void onItemSelected(AdapterView<?> parent, View view, int pos, long id) {
        String selected = parent.getItemAtPosition(pos).toString();
        synchronized (fisheyeImageLock) {
            targetFrameRate = Double.parseDouble(selected);
            // restart statistics and rate shaping
            startingTimeStamp = -1.0;
            globalSlot = 0;
            framesProcessed = 0;
        }
    }

    public void onNothingSelected(AdapterView parent) {
        // Do nothing.
    }
    // Project Tango Service connection.
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            TangoJniNative.setBinder(service);
            TangoJniNative.setupConfig();
            TangoJniNative.connectCallbacks();
            TangoJniNative.connect();

            if (threadsStarted) {
                return;
            }

            for (int i = 0; i < imagesFisheyeThread.length; i++) {
                imagesFisheyeThread[i] = new Thread(new Runnable() {
                    public void run() {
                        while (true) {
                            final double ts;
                            boolean processFrame = false;

                            synchronized (fisheyeImageLock) {
                                ts = TangoJniNative.getFisheyeFrameTimestamp();
                                // this is not quite thread safe... need to synchronize the check and assignment
                                int slot = (int) Math.floor(ts * targetFrameRate);
                                if (slot > globalSlot) {
                                    globalSlot = slot;
                                    if (startingTimeStamp == -1.0) {
                                        startingTimeStamp = ts;
                                    }
                                    processFrame = true;
                                }
                            }
                            if (!processFrame) {
                                // it would be better to have some sort of signal sent to us, but
                                // instead we'll just sleep for a bit to avoid checking too frequently
                                try {
                                    Thread.sleep(5);
                                } catch (InterruptedException ex) {
                                    // thread was interrupted... no big deal
                                }
                                continue;
                            }
                            final int[] fisheyeStride = new int[1];
                            final byte[] fisheyePixels = new byte[fisheyeImageWidth*fisheyeImageHeight*3/2];
                            final double[] tagDetection = new double[8];          // 4 points with 2 coordinates each

                            // grab the pixels and any tag detections
                            TangoJniNative.returnArrayFisheye(fisheyePixels, fisheyeStride, tagDetection);
                            framesProcessed++;
                            int startSlot = (int) Math.floor(startingTimeStamp*targetFrameRate);
                            final double frameRateRatio = framesProcessed/((float)globalSlot - startSlot);
                            System.out.println("Frame rate goal " + targetFrameRate + " ratio " + frameRateRatio);

                            synchronized (updateImageViewLock) {
                                if (ts < lastDisplayedImageTS) {
                                    // no need to display the image, there is already a more recent one that has been displayed
                                    continue;
                                }
                                // mark that we are going to display this image, and put the display of it on the UI event queue
                                lastDisplayedImageTS = ts;

                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        TextView textView = (TextView) findViewById(R.id.frame_rate_text);
                                        textView.setText("Actual frame rate: " + String.format("%.1f", frameRateRatio*targetFrameRate));
                                        // the fisheye image uses a stride that is not the same as the image width
                                        int[] strides = {fisheyeStride[0], fisheyeStride[0]};
                                        YuvImage fisheyeFrame = new YuvImage(fisheyePixels,
                                                                             android.graphics.ImageFormat.NV21,
                                                                             fisheyeImageWidth,
                                                                             fisheyeImageHeight,
                                                                             strides);
                                        // somewhat hacky method of getting the YUVImage to a BitMap
                                        ByteArrayOutputStream out = new ByteArrayOutputStream();
                                        fisheyeFrame.compressToJpeg(new Rect(0, 0, fisheyeImageWidth, fisheyeImageHeight), 80, out);
                                        byte[] imageBytes = out.toByteArray();
                                        Bitmap image = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
                                        Bitmap mutableBitmap = image.copy(Bitmap.Config.ARGB_8888, true);

                                        if (tagDetection[0] >= 0.0) {
                                            Canvas canvas = new Canvas(mutableBitmap);

                                            Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
                                            paint.setColor(Color.rgb(255, 0, 0));
                                            paint.setStrokeWidth(4.0f);
                                            for (int j = 0; j < 4; j++) {
                                                canvas.drawLine((float) tagDetection[(2 * j) % tagDetection.length],
                                                        (float) tagDetection[(2 * j + 1) % tagDetection.length],
                                                        (float) tagDetection[(2 * j + 2) % tagDetection.length],
                                                        (float) tagDetection[(2 * j + 3) % tagDetection.length],
                                                        paint);
                                            }
                                        }
                                        ImageView iv = (ImageView) findViewById(R.id.fisheye_image);
                                        iv.setImageBitmap(rotateBitmap(mutableBitmap, -90.0f));
                                    }
                                });
                            }
                        }
                    }
                });
                imagesFisheyeThread[i].start();
            }
            threadsStarted = true;
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

        Spinner frameRateSpinner = (Spinner)findViewById(R.id.frame_rate_spinner);
        Integer[] items = new Integer[30];
        for (int i = 0; i < items.length; i++) {
            items[i] = i+1;
        }

        ArrayAdapter<Integer> adapter = new ArrayAdapter<Integer>(this,android.R.layout.simple_spinner_item, items);
        frameRateSpinner.setAdapter(adapter);
        frameRateSpinner.setSelection(((int)targetFrameRate) - 1);      // subtract 1 since setSelection is by index, not by value
        frameRateSpinner.setOnItemSelectedListener(this);
    }

    @Override
    protected void onResume() {
        super.onResume();
        TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        // TODO: need to handle this properly for cases when the activity is suspended by Android (e.g., when plugging into the charger)
        // Always call the superclass so it can save the view hierarchy state
        super.onSaveInstanceState(savedInstanceState);
    }

    @Override
    protected void onStop() {
        super.onStop();
    }

    @Override
    protected void onPause() {
        super.onPause();
        TangoJniNative.disconnect();
        unbindService(mTangoServiceConnection);
    }
}