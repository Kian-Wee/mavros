#!/bin/bash

# Setup script for borealis, adds aliases and copies parallel commands script for simultaneous launching
# Not stable at the moment, copy paste

# Prompt user for drone name
echo Enter Drone Number[eg, 0]
read DRONE_NUMBER_TEMP
echo The Drone Number is $DRONE_NUMBER_TEMP , edit DRONE_NUMBER in bashrc to change
export DRONE_NAME_TEMP="uav"$DRONE_NUMBER_TEMP
echo The Drone Name is $DRONE_NAME_TEMP , edit DRONE_NAME in bashrc to change

# Install dependancies
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera -y
sudo apt-get install ntpdate -y
sudo apt-get install libgeographic-dev ros-melodic-geographic-msgs geographiclib-tools -y



# Install buzzer package
cd ~/catkn_ws/src/
git clone https://github.com/Kian-Wee/PX4-Lights
git clone 

# Copy bash script for running multiple commands into home folder
cp ~/catkin_ws/src/mavros/borealis_setup/parallel_comments.bash ~

# Copy aliases
cat >> ~/.bashrc << EOF	
source ~/catkin_ws/devel/setup.bash --extend

# Configure these
#export DRONE_NAME="uav0"
export DRONE_NUMBER=$DRONE_NUMBER_TEMP #Remember to change the gcs_url port in the  launch file too
export DRONE_NAME=$DRONE_NAME_TEMP #Remember to change the gcs_url port in the  launch file too
export GCS_PORT=14555 #Remember to change the gcs_url port in the  launch file too
export T265_ID="948422110423"
export D435_ID="134222075005"

export UAV_X="0"
export UAV_Y="0"
export UAV_Z="0"

export ROS_MASTER_URI=http://127.0.0.1:11311/ #set to host
export ROS_HOSTNAME=127.0.0.1 #always set as own ip
export ROS_IP=127.0.0.1 #always set as own ip

## Realsense aliases
alias launch_d435i='roslaunch realsense2_camera rs_camera.launch camera:=$DRONE_NAME/d435i serial_no:=$D435_ID color_width:=848 color_height:=480 color_fps:=30 depth_width:=848 depth_height:=480 depth_fps:=30 align_depth:=true initial_reset:=true enable_sync:=true'
alias launch_t265='roslaunch realsense2_camera rs_t265.launch serial_no:=$T265_ID camera:=$DRONE_NAME/t265'

## Ouster aliases
alias launch_ouster='roslaunch ouster_ros ouster.launch sensor_hostname:="os-122121000737.local"  metadata:="/home/borealis/ouster/ouster_example/ouster_meta.json" '
#alias launch_ouster='roslaunch ouster_ros ouster.launch sensor_hostname:="os1-991942000601.local"  metadata:="/home/borealis/ouster/ouster_example/ouster_meta.json" '

## Mavros aliases
# Syncronise time and launch mavros, 2 realsenses, the positioning node and the LEDs
alias launch_borealis='. ~/parallel_comments.bash "sudo ntpdate -s time.nist.gov" "roslaunch mavros px4_swarm.launch" "roslaunch realsense2_camera rs_t265.launch camera:=$DRONE_NAME/t265 serial_no:=$T265_ID --wait" "rosrun mavros pos_265" "roslaunch realsense2_camera rs_camera.launch camera:=$DRONE_NAME/d435i serial_no:=$D435_ID color_width:=848 color_height:=480 color_fps:=30 depth_width:=848 depth_height:=480 depth_fps:=30 align_depth:=true initial_reset:=true enable_sync:=true --wait" "python ~/catkin_ws/src/PX4-Lights/buzzer_ros.py"'
# Alternative launch script using combined roslaunch
alias launch_borealis2='. ~/parallel_comments.bash "sudo ntpdate -s time.nist.gov" "roslaunch mavros px4_swarm_realsense.launch" "python ~/catkin_ws/src/PX4-Lights/buzzer_ros.py"'
# Launch Borealis with Ouster tracking, also launches Ouster
alias launch_borealis_ouster='. ~/parallel_comments.bash "sudo ntpdate -s time.nist.gov" "roslaunch mavros px4_swarm.launch" "roslaunch realsense2_camera rs_t265.launch camera:=$DRONE_NAME/t265 serial_no:=$T265_ID --wait" "rosrun mavros pos_265_ouster" "roslaunch realsense2_camera rs_camera.launch camera:=$DRONE_NAME/d435i serial_no:=$D435_ID color_width:=848 color_height:=480 color_fps:=30 depth_width:=848 depth_height:=480 depth_fps:=30 align_depth:=true initial_reset:=true enable_sync:=true --wait" "python ~/catkin_ws/src/PX4-Lights/buzzer_ros.py" "roslaunch ouster_ros ouster.launch sensor_hostname:="os1-991942000601.local"  metadata:="/home/borealis/ouster/ouster_example/ouster_meta.json" --wait" "roslaunch aloam_velodyne aloam_ouster_32_test.launch --wait"'

sudo ntpdate -s time.nist.gov
EOF

# Add in udev rules, chmod of this script required for editing protected files
sudo chmod 777 ~/catkin_ws/src/mavros/borealis_setup/setup.bash
sudo touch "/etc/udev/rules.d/99-pixhawk.rules"
sudo bash -c 'cat << EOF > /etc/udev/rules.d/99-pixhawk.rules
# Pixhawk 4
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0032", SYMLINK+="ttyPixhawk"
# Pixhawk 2
#SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyPixhawk"
EOF'

sudo usermod -a -G tty borealis
sudo usermod -a -G dialout borealis

echo "Please reboot if permissions were not set before"



# PLANNER SETUP
echo "SETTING UP PLANNER"

# Acados setup, ignore/delete if not needed https://docs.acados.org/installation/index.html
cd ~
git clone https://github.com/acados/acados.git
cd ~/acados
git submodule update --recursive --init

# Copy aliases
cat >> ~/.bashrc << EOF
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/$USER/acados/lib"
export ACADOS_SOURCE_DIR="/home/$USER/acados"
EOF

source ~/.bashrc

# Python installation, source: https://discourse.acados.org/t/acados-installation-in-pycharm/103/11
sudo apt-get install virtualenv -y
sudo apt install python3-pip -y
pip3 install -e /home/$USER/acados/interfaces/acados_template

echo "Build Acados? [Y,n]"
read input
if [[ $input == "Y" || $input == "y" ]]; then
    echo "building acados"

	# echo "Since some of the C examples use qpOASES, also set ACADOS_WITH_QPOASES = 1 in <acados_root_folder>/Makefile.rule"	
	
	# IGNORE SUBSEQUENT BUILD IF NOT MAKING FOR EXACT PLATFORM
	cd ~/acados
	rm build/* -rf
	mkdir build
	cd build
	cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_EXAMPLES=ON -DHPIPM_TARGET=GENERIC -DBLASFEO_TARGET=GENERIC
	make -j4
	make install -j4

	cd ~/acados
	make shared_library
	make examples_c
	make run_examples_c

	# run a C example, e.g.:
	cd build
	./examples/c/sim_wt_model_nx6

	# Python example
	echo "Ensure X-11 forwarding is enabled when running over ssh"
	cd ~/acados/examples/acados_python/getting_started
	python3 minimal_example_closed_loop.py
else
        echo "Not building acados"
fi
