alias matlab='cd /usr/local/MATLAB/R2020b/bin; sudo ./matlab'

alias export_drone4='export ROS_MASTER_URI=http://192.168.1.40:11311/; export ROS_HOSTNAME=192.168.1.3; export ROS_IP=192.168.1.3'

alias px4_gazebo='roslaunch mavros px4_swarm_extra.launch fcu_url:="udp://:14540@127.0.0.1:14557"' # "udp://:14540@192.168.1.36:14557"
alias mavros_gazebo='cd ~/Firmware/Firmware; DONT_RUN=1 make px4_sitl_default gazebo; source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd); export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo; roslaunch px4 posix_sitl.launch '
## DOESNT WORK
#alias drone_sim='px4_gazebo & mavros_gazebo && fg'
## Parallel commands dont work with aliases as commands
# alias drone_sim='. ~/parallel_comments.bash "roslaunch mavros px4_group.launch fcu_url:="udp://:14540@192.168.1.36:14557"" "cd ~/Firmware/Firmware; DONT_RUN=1 make px4_sitl_default gazebo; source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd); export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo; roslaunch px4 posix_sitl.launch "'
#. ~/parallel_comments.bash "roslaunch mavros px4_2.launch" "roslaunch realsense2_camera rs_t265.launch camera:=t265 serial_no:=948422110423" "rosrun mavros pos_265"

# Terminator Macros
alias terminator='terminator -l default'
alias borealis='terminator -l borealis'
alias borealis_hri='terminator -l borealis_hri'	

# SSH Macros - Direct ssh without prompt with sshpass
alias sshborealis='sshpass -p 748748748 ssh -o StrictHostKeyChecking=no borealis@192.168.1.10'
alias sshborealis2='sshpass -p 748748748 ssh -o StrictHostKeyChecking=no borealis-2@192.168.1.20'
alias sshborealis3='sshpass -p 748748748 ssh -o StrictHostKeyChecking=no borealis-3@192.168.1.30'
alias sshhuman='sshpass -p 12345 ssh -o StrictHostKeyChecking=no sutd@192.168.1.50'
alias sshhuman2='sshpass -p 12345 ssh -o StrictHostKeyChecking=no Ril123@192.168.1.24'
alias sshpx4vision='sshpass -p 748748748 ssh -o StrictHostKeyChecking=no px4vision@192.168.1.60'

#alias sshborealis3='sshpass -p 748748748 ssh -o StrictHostKeyChecking=no borealis-3@192.168.1.30'
#alias sshborealis2='sshpass -p 748748748 ssh -o StrictHostKeyChecking=no borealis-2@192.168.1.98'
#alias sshborealis='sshpass -p 748748748 ssh -o StrictHostKeyChecking=no borealis@192.168.1.40'
#alias sshhuman='sshpass -p 12345 ssh -o StrictHostKeyChecking=no sutd@192.168.1.50'
#alias sshhuman2='sshpass -p 12345 ssh -o StrictHostKeyChecking=no Ril123@192.168.1.24'
#alias sshpx4vision='sshpass -p 748748748 ssh -o StrictHostKeyChecking=no px4vision@192.168.1.12'

## This works
#sshpass -p 748748748 ssh -o StrictHostKeyChecking=no borealis@192.168.1.40 'hostname'
## This doesnt work cause ros requires special setup for non remote launching
#sshpass -p 748748748 ssh -o StrictHostKeyChecking=no borealis@192.168.1.40 'roslaunch realsense2_camera rs_t265.launch camera:=cam2 serial_no:=8122110233'

# Program Macros
alias setup1='roscore'
alias setup2='./QGroundControl.AppImage'
alias setup3='terminator -l borealis_hri'
alias setup4='rqt'

: <<'END_COMMENT'

# Borealis Macros
alias 0.1='roscleanpurge'
alias 0='rossource; roslaunch ~/ROS_Starter/starter.bash' #for roscore
alias 1='launch_t265'#'roslaunch realsense2_camera rs_t265.launch'
alias 2='sleep 2;roslaunch mavros px4_2.launch'
alias 3='launch_d435' #'roslaunch realsense2_camera rs_camera.launch' #d435i
alias 4='sleep 4;rosrun mavros pos_265'
alias 5='python buzzer_ros.py'
alias 6='rostopic echo /uav1/targepose'
alias 7='launch_drone_data_feeder' #launch drone_cv
alias 8='launch_rifle_node'
alias 9='rostopic pub /hri_user_input std_msgs/Int32 1'

## Borealis 1 realsense serial Macros
#alias launch_d435i='roslaunch realsense2_camera rs_camera.launch camera:=cam1 serial_no:=036522070595'
#alias launch_t265='roslaunch realsense2_camera rs_t265.launch camera:=cam2 serial_no:=948422110423'
#alias launch_both_cam='launch_d435i & launch_t265'

## Borealis 3 realsense
#alias launch_d435i='roslaunch realsense2_camera rs_camera.launch camera:=cam1 serial_no:=040322072991'
#alias launch_t265='roslaunch realsense2_camera rs_t265.launch camera:=cam2 serial_no:=132222111113'

# Human Macros
#alias ='rqt'
#alias hri1='launch_rifle_node'
#alias hri2='roslaunch drone_data_feeder_node"

#apt-get install ntpdate
#sudo ntpdate -s time.nist.gov

END_COMMENT
