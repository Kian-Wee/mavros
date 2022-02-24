#!/bin/bash

# Setup script for borealis, adds aliases and copies parallel commands script for simultaneous launching

# Install dependancies
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera -y
sudo apt-get install ntpdate -y

# Copy bash script for running multiple commands into home folder
cp ~/catkin_ws/src/mavros/borealis_setup/parallel_comments.bash ~

# Copy aliases
cat >> ~/.bashrc << EOF
source ~/catkin_ws/devel/setup.bash

# Configure these
export DRONE_NAME="uav0"
export T265_ID="948422110423"
export D435_ID="134222075005"

export ROS_MASTER_URI=http://127.0.0.1:11311/ #set to host
export ROS_HOSTNAME=127.0.0.1 #always set as own ip
export ROS_IP=127.0.0.1 #always set as own ip

# Realsense aliases
alias launch_d435i='roslaunch realsense2_camera rs_camera.launch camera:=$DRONE_NAME/d435i serial_no:=$D435_ID color_width:=848 color_height:=480 color_fps:=30 depth_width:=848 depth_height:=480 depth_fps:=30 align_depth:=true'
alias launch_t265='roslaunch realsense2_camera rs_t265.launch serial_no:=$T265_ID camera:=$DRONE_NAME/t265'

alias launch_drone_data_feeder='roslaunch drone_data_feeder_node drone_data_feeder.launch'
alias drone_data_feeder='rosrun drone_data_feeder_node drone_data_feeder.py'

# Syncronise time and launch mavros, 2 realsenses, the positioning node and the LEDs
alias launch_borealis='. ~/parallel_comments.bash "sudo ntpdate -s time.nist.gov" "roslaunch mavros px4_swarm.launch" "roslaunch realsense2_camera rs_t265.launch camera:=$DRONE_NAME/t265 serial_no:=$T265_ID" "rosrun mavros pos_265" "roslaunch realsense2_camera rs_camera.launch camera:=$DRONE_NAME/d435i serial_no:=$D435_ID color_width:=848 color_height:=480 color_fps:=30 depth_width:=848 depth_height:=480 depth_fps:=30 align_depth:=true" "python ~/catkin_ws/src/PX4-Lights/buzzer_ros.py'
EOF

# Add in udev rules
sudo touch "/etc/udev/rules.d/99-pixhawk.rules"
sudo bash -c 'cat << EOF > /etc/udev/rules.d/99-pixhawk.rules
# Pixhawk 4
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixhawk"
# Pixhawk 2
#SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyPixhawk"
EOF'

sudo usermod -a -G tty borealis
sudo usermod -a -G dialout borealis

echo "Please reboot if permissions were not set before"



# Acados setup, ignore/delete if not needed https://docs.acados.org/installation/index.html
cd ~
git clone https://github.com/acados/acados.git
git submodule update --recursive --init
# source https://discourse.acados.org/t/acados-installation-in-pycharm/103/11
cd ~/acados
rm build/* -rf
cd build
cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_EXAMPLES=ON -DHPIPM_TARGET=GENERIC -DBLASFEO_TARGET=GENERIC
make -j4
make install -j4

# Copy aliases
cat >> ~/.bashrc << EOF
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/kw/acados/lib"
export ACADOS_SOURCE_DIR="/home/kw/acados"
EOF

source ~/.bashrc

# run a C example, e.g.:
./examples/c/sim_wt_model_nx6

cd ~/acados
make examples_c
make run_examples_c
make shared_library




