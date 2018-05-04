# Streaming Sensor Data in order to Rapidly Prototype Augmented Reality Apps

`tango_ros_bridge` allows you to stream sensor data from a Google Tango smartphone directly to any computer running ROS.  Streaming data in this manner allows you to program apps the utilize the rich sensor data of Tango (fisheye camera, 3D location tracking, and 3D point clouds) using any of the programming languages supported by ROS (Python and C++).  Additionally, you will be able to leverage all of the powerful features of ROS (e.g., visualization tools, debugging tools, recording tools) from the comfort of your own laptop (or other computer).

![A screenshot showing 3D data from a Tango smartphone being displayed in RVIZ, a visualization program for ROS](images/tango_ros_bridge_screenshot_1.png)

# Getting Started
`ros_tango_bridge` has two components you will need to configure: an Android app that runs on your Tango smartphone and a ROS package that should be installed on your ROS computer.

## Prerequisites

In order to use `tango_ros_streamer`, you must have the following dependencies installed.
* [ROS (Indigo or later)](http://wiki.ros.org/ROS/Installation)
* [Android Studio](https://developer.android.com/studio/index.html)

## Obtaining the Code

To obtain the code clone the `ros_tango_bridge` repository.  To streamline the integration of the code from the repository with ROS, you should clone the repository into your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) `src` directory.  For instance, if your catkin workspace is installed in the typical location, you would use these commands to clone the repository.

``` bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/occamLab/tango_ros_bridge
```

## Building and Installing the Android App

An [Android Studio](https://developer.android.com/studio/index.html) project is included as part of the repository under the `android_code` subdirectory.  Use Android Studio to build and install the app to your Tango-equipped smartphone.

## Installing the ROS package

As long as you cloned the repository into your catkin workspace, there is nothing you have to do in order to install the ROS package.  You do, however, have to install one Python library.  The `decorator` library can be installed using `pip`.

``` bash
$ sudo pip install decorators
```

## Running tango_ros_bridge

On the machine running ROS, launch the main `tango_ros_streamer` launch file.
``` bash
$ roslaunch tango_streamer stream.launch
```

On the smartphone, open up the `ROS Streamer` app, enter the IP address of your ROS computer and click `Connect`.

To ensure that the connection has been made properly, on the computer running ROS print out the pose information from the phone.
``` bash
$ rostopic echo /tango_pose
```

If all goes well, you should see output similar to the following.

```
---
header:
  seq: 619
  stamp:
    secs: 1524579566
    nsecs: 828507900
  frame_id: "odom"
pose:
  position:
    x: -0.0494725894225
    y: 0.0452275977115
    z: 0.173853426765
  orientation:
    x: 0.0352902299281
    y: 0.1361060735
    z: -0.0701101320277
    w: 0.987580025018
---
header:
  seq: 620
  stamp:
    secs: 1524579566
    nsecs: 853507995
  frame_id: "odom"
pose:
  position:
    x: -0.0495403503256
    y: 0.0454651608819
    z: 0.173958186925
  orientation:
    x: 0.0350289289155
    y: 0.136183373272
    z: -0.0704992675044
    w: 0.987550968945
```

# Software Architecture

`tango_ros_bridge` consists of two main components: an Android app and a ROS package.

![The high-level architecture of ros_tango_bridge.  Details are available in the figure caption](images/tango_ros_bridge_architecture.png)

The architecture diagram shows the interplay between the Android app and the ROS package.  The two components communicate over an IP-based network using both UDP and TCP sockets.  The ros package converts the raw sensor data to an appropriate ROS message and publishes it in on a topic so that it can be consumed by client ROS nodes.

# Sample Use Cases
Here are some instances of `ros_tango_bridge` in action.

## Sparse 3D Mapping of a Building

![Corrected (red) and uncorrected (blue) paths through Olin's academic center](images/ac_map.jpg)

William Derksen utilized this platform to build a 3D map of Olin's academic center.  The image below shows a side view of two paths through all four floors of the academic center.  The *red* path is the raw sensor data as determined by the Tango phone and the *blue path* is a corrected version that utilized special optical landmarks for drift correction.

## Simple Dense 3D Mapping

Outline of content to be generated:
* Create a screen capture of creating a rough 3D map using `rviz` and raw point cloud data.
* The screen capture will show the phone moving around in space, with point clouds being projected using `rviz`. By the end of the video, the structure of the room will be clearly visible.

# Project History

This code was developed starting in 2015 with the aim of allowing Olin students to easily program systems based on the Tango platform.  In particular, `ros_tango_bridge` is designed to smooth the learning curve that would be needed to program directly on the phone itself (e.g., learning Java, learning Android Studio, etc.).

The initial version of the code supported the Yosemite development Kit supplied by Google.  The current version works with the Lenovo Phab2 Pro (although, adapting it to other platforms can be done with a modest amount of work).

# Contributors
The bulk of this code was generated by [Paul Ruvolo](https://github.com/paulruvolo).   [Dhash Shrivathsa](https://github.com/DhashS) contributed to the integration of UDP streaming.

# License
Copyright 2018 Paul Ruvolo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
