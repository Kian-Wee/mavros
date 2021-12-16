#!/bin/bash

# Setup script for borealis, adds aliases and copies parallel commands script for simultaneous launching

cp ~/catkin_ws/src/mavros/borealis_setup/parallel_comments.bash ~

cat >> ~/.bashrc << EOF
alias launch_d435i='roslaunch realsense2_camera rs_camera.launch camera:=uav0/d435i serial_no:=134222075005 color_width:=848 color_height:=480 color_fps:=30 depth_width:=848 depth_height:=480 depth_fps:=30 align_depth:=true'
alias launch_t265='roslaunch realsense2_camera rs_t265.launch serial_no:=948422110423 camera:=uav0/t265'

alias launch_drone_data_feeder='roslaunch drone_data_feeder_node drone_data_feeder.launch'
alias drone_data_feeder='rosrun drone_data_feeder_node drone_data_feeder.py'

alias launch_borealis='. ~/parallel_comments.bash "roslaunch mavros px4_swarm_extra.launch" "roslaunch realsense2_camera rs_t265.launch camera:=uav0/t265 serial_no:=948422110423" "rosrun mavros pos_265" "roslaunch realsense2_camera rs_camera.launch camera:=uav0/d435i serial_no:=134222075005 color_width:=848 color_height:=480 color_fps:=30 depth_width:=848 depth_height:=480 depth_fps:=30 align_depth:=true"'
EOF
