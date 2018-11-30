# Final_Robotics_Project

This project implements autonomous navigation of turtlebot2 in unknown maze using wall follower algorithm and hector_slam. The maze contains objects, which will be detected by RealSense2 Camera. 

Once detected, The robot will also be able to navigate to a given object autonomously.

## Installation

These steps are needed to perform only for fresh development on new machine.

*Intel RealSense2 camera
To use the intel realsense in ROS, see: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

*hls_lfcd_lds 360 laser scanner
Follow the installation steps given in: https://github.com/rawanazim/hls_lfcd_lds_driver

This will install hls driver in the system, Use this in place of Hokyuo scanner (For verifying the SLAM with this, Just run hector_slam_launch tutorial.launch)
