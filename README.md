# Robotics Final Project


### Requirements

follow the realsense installation I fixed. Make sure this can run: `roslaunch realsense2_camera rs_rgbd_low_fps.launch`


### Realsense Needed Topics

for the depth info: `/camera/aligned_depth_to_color/image_raw`

for the color info: `/camera/color/image_raw`


### Running:

this needs to be run to start:

`roslaunch realsense2_camera rs_rgbd.launch`


it usually helps to open `rviz` and change the fixed frame to `camera_link`
ADD the following by topic:
* /camera/aligned_depth_to_color/image_raw
* /camera/color/image_raw
* /camera/depth_registered/points


to run the square recognition:

`./scripts/sense_dude.py`

this publishes to `/sense_dude_square_loc`

