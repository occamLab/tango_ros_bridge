# Streaming Sensor Data to Rapidly Prototype Augmented Reality Apps

`tango-ros-bridge` allows you to stream sensor data from a Google Tango smartphone directly to any computer running ROS.  Streaming data in this manner allows you to program apps the utilize the rich sensor data of Tango (fisheye camera, 3D location tracking, and 3D point clouds) using any of the programming languages supported by ROS (Python and C++).  Additionally, you will be able to leverage all of the powerful features of ROS (e.g., visualization tools, debugging tools, recording tools) from the comfort of your own laptop (or other computer).


# Getting Started
`ros_tango_bridge` has two components you will need to configure: an Android app that runs on your Tango smartphone and a ROS package that should be installed on your ROS computer.

In preparation for these steps, you should clone `ros_tango_bridge` repository.  To streamline the integration of the code from the repository with ROS, you should clone the repository into your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) `src` directory.  For instance, if your catkin workspace is installed in the typical location, you would use these commands to clone the repository.

``` bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/occamLab/tango_ros_bridge
```



## Building and Installing the Android App


## Installing the ROS package



There are many successful formats for a project website, but you should consider including:

Big Idea/Goal/What is this?/ Why did we do this? Quick and easily understandable explanation of what your project is all about. Consider including a narrative or example use case, e.g. via screenshots, video, or story boarding.

User instructions/README Information to help users download, install, and get started running your software (README rubric)

Implementation information Code doesn’t tell a story by itself. Use more effective methods such as flowcharts and architectural, class, or sequence diagrams to explain how your code works. You could consider including or linking to snippets of code to highlight a particularly crucial segment.

Results Though the details will be different for each project, show off what your software can do! Screenshots and video are likely helpful. Include graphs or other data if appropriate.

Project evolution/narrative Tell an illustrative story about the process of creating your software, showing how it improved over time. This may draw upon what you learned from the two peer technical reviews and from the code review. Consider the use of screenshots or other tools to demonstrate how your project evolved.

Attribution Give proper credit for external resources used.
