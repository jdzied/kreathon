/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.example.roscameratestapp;

import android.hardware.Camera;
import android.os.Bundle;
import android.os.Handler;
import android.view.MotionEvent;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

/**
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 */

public class MainActivity extends RosActivity {

    private int cameraId = 0;
    private RosCameraPreviewView rosCameraPreviewView;
    private Handler handy = new Handler();

    public MainActivity() {
        super("CameraTutorial", "CameraTutorial");
    }

    Runnable sizeCheckRunnable = new Runnable() {
        @Override
        public void run() {
            if (rosCameraPreviewView.getHeight()== -1 || rosCameraPreviewView.getWidth()== -1) {
                handy.postDelayed(this, 100);
            } else {
                Camera camera = Camera.open(cameraId);
                rosCameraPreviewView.setCamera(camera);
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);
        rosCameraPreviewView = (RosCameraPreviewView) findViewById(R.id.ros_camera_preview_view);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeConfiguration.setNodeName(rosCameraPreviewView.getDefaultNodeName() + "_" + InetAddressFactory.newNonLoopback().getHostAddress().replace(".",""));
        nodeMainExecutor.execute(rosCameraPreviewView, nodeConfiguration);
        handy.post(sizeCheckRunnable);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_UP) {
            int numberOfCameras = Camera.getNumberOfCameras();
            final Toast toast;
            if (numberOfCameras > 1) {
                cameraId = (cameraId + 1) % numberOfCameras;
                rosCameraPreviewView.releaseCamera();
                rosCameraPreviewView.setCamera(Camera.open(cameraId));
                toast = Toast.makeText(this, "Switching cameras.", Toast.LENGTH_SHORT);
            } else {
                toast = Toast.makeText(this, "No alternative cameras to switch to.", Toast.LENGTH_SHORT);
            }
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    toast.show();
                }
            });
        }
        return true;
    }

}