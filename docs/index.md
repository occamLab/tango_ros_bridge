# Streaming Sensor Data to Rapidly Prototype Augmented Reality Apps

`tango_ros_bridge` allows you to stream sensor data from a Google Tango smartphone directly to any computer running ROS.  Streaming data in this manner allows you to program apps the utilize the rich sensor data of Tango (fisheye camera, 3D location tracking, and 3D point clouds) using any of the programming languages supported by ROS (Python and C++).  Additionally, you will be able to leverage all of the powerful features of ROS (e.g., visualization tools, debugging tools, recording tools) from the comfort of your own laptop (or other computer).

TODO: add some screen shots

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


User instructions/README Information to help users download, install, and get started running your software (README rubric)

Implementation information Code doesn’t tell a story by itself. Use more effective methods such as flowcharts and architectural, class, or sequence diagrams to explain how your code works. You could consider including or linking to snippets of code to highlight a particularly crucial segment.

Results Though the details will be different for each project, show off what your software can do! Screenshots and video are likely helpful. Include graphs or other data if appropriate.

Project evolution/narrative Tell an illustrative story about the process of creating your software, showing how it improved over time. This may draw upon what you learned from the two peer technical reviews and from the code review. Consider the use of screenshots or other tools to demonstrate how your project evolved.

Attribution Give proper credit for external resources used.
