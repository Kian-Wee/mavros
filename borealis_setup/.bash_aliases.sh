## Launch these 2 nodes for sitl with ros
alias px4_gazebo='roslaunch mavros px4_swarm_extra.launch fcu_url:="udp://:14540@127.0.0.1:14557"' # "udp://:14540@192.168.1.36:14557"
alias mavros_gazebo='cd ~/Firmware/Firmware; DONT_RUN=1 make px4_sitl_default gazebo; source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd); export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo; roslaunch px4 posix_sitl.launch '

# Launch multiple drones sitl in ros
alias multi_gazebo='cd ~/Firmware/Firmware; git submodule update --init --recursive; DONT_RUN=1 make px4_sitl_default gazebo; source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo;roslaunch px4 multi_uav_mavros_sitl.launch'

