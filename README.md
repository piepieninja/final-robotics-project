# Final_Robotics_Project

This project implements autonomous navigation of turtlebot2 in unknown maze using wall follower algorithm and hector_slam. The maze contains objects, which will be detected by RealSense2 Camera. 

Once detected, The robot will also be able to navigate to a given object autonomously.

## Installation

These steps are needed to perform only for fresh development on new machine which is on the robot.

* Intel RealSense2 camera

To use the intel realsense in ROS, see: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

* hls_lfcd_lds 360 laser scanner

Follow the installation steps given in: https://github.com/rawanazim/hls_lfcd_lds_driver

This will install hls driver in the system, Use this in place of Hokyuo scanner (For verifying the SLAM with this, Just run hector_slam_launch tutorial.launch)

## Running the demo

* Prerequisites:

1) Working ROS on remote as well as onboard computer.
2) Working network connection between both the machines.(http://wiki.ros.org/ROS/NetworkSetup)
3) Above hardware setup is working on onboard machine.
4) Hector-SLAM installed on Remote machine.
5) Make sure you can access all the published topics on the remote machine.(If not try resolving the ssh error described in any of the solutions here: https://answers.ros.org/question/90536/ros-remote-master-can-see-topics-but-no-data/?answer=90956#post-id-90956 )

Once done, Place Turtlebot anywhere in the world, and Run the following:

Start the ``` roscore``` on master machine.

```
roslaunch final-robotics-project team_c.launch
```

This will launch Turtlebot bringup, HLS driver on the onboard machine and Wall follower, Square Detection, and Hector SLAM on remote machine.

## Observations

The robot starts in Wall Detection mode, and once identified the farthest possible wall, Starts approaching the starting point of the wall. Then it follows the wall based on laser scan data and avoids obstacles in front of the path.

Once the whole wall is followed and loop is detected, the robot senses for new wall and repeats the above sequence.

Using the edge detection the program identifies the squares of Red, Blue, Green colors. The camera gives the depthe of the image detected. Then it is used to detect the position of square with respect to camera frame.

This position will be transformed to world frame (with the help of map generated using HectorSLAM) and stored in a feature map (YET TO BE IMPLEMENTED).

## Results
