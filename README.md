# BOREALIS SPECIFIC MAVROS SETUP

## **Migrating From existing Borealis drone**
change the DRONE_NAME variable in bashrc to a new drone id
### Change hostname
sudo nano /etc/hostname , delete and change to a new name
sudo nano /etc/hosts , replace all existing computer name with new one
sudo reboot

### Rebuild workspaces

You might need to catkin clean and catkin build the workspaces again


## **New Borealis Setup**

### Installation
Follow inital mavros installation and change branch to pull changes

Initalise this fork from master branch

	1. run through normal ros and mavros setup
	
	2. git remote set-url origin https://github.com/Kian-Wee/mavros.git
	
	3. git branch borealis
	
	4. git checkout borealis
	
	5. git pull origin borealis

If any prior changes were made, drop the changes first with, ```git stash```, ```git stash drop stash@{0}```. Enter name and email if prompted.

More instructions(on git) available at [Github & Syntax & Bash](Github & Syntax & Bash/Github Updating Instructions)



### **Vehicle and position setup** 
Run the setup file to initalise aliases and udev rules
``` . ~/catkin_ws/src/mavros/borealis_setup/setup.bash ```

#### PX4 Params configuration
EKF2_HGT_Mode -> Vision
EKF2_RNG_AID -> Enabled
EKF_2_AID_MASK -> Vision position fusion, Vision yaw fusion (uncheck everything else)
[Developer blogpost on params](https://hubs.la/Q0168CVX0)

#### QGC Setup
Do remember to change the MAV_SYS_ID on different vehicles, subsequently, change the gcs url port in the mavros launch file
The first drone using port 14555 and the subsequent starting from 14556(or any port your prefer), Remember to add the port in QGC when monitoring

[Other reference](https://github.com/PX4/PX4-Devguide/blob/master/en/companion_computer/pixhawk_companion)

To run the ouster, ensure the ethernet link is set to link-local only under ipv4 settings



### **Scripts**

**basic_position** - send positions and generate trajectories

**pos_265** - node for Intel Realsense 265, visual positioning is given to mavros (pos_265_old shows all output)

**follow+odom+velocity** - follows and republishes odom and velocity, counts error per lap

### **Old Scripts**

**follow** - basic follow script to follow a topic

**follow+** - saves additional datapoints

**follow+odom** - uses odometry instead of posestamped and implements a lapcounter(for use with basic_position)

### Multi-Drone Setup
Run ``` sudo ntpdate -s time.nist.gov ``` on all devices


MAVROS
======
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/mavlink/mavros)](https://github.com/mavlink/mavros/releases)  [![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/mavros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)  [![CI](https://github.com/mavlink/mavros/actions/workflows/main.yml/badge.svg)](https://github.com/mavlink/mavros/actions/workflows/main.yml)

MAVLink extendable communication node for ROS.

- Since 2014-08-11 this repository contains several packages.
- Since 2014-11-02 hydro support separated from master to hydro-devel branch.
- Since 2015-03-04 all packages also dual licensed under terms of BSD license.
- Since 2015-08-10 all messages moved to mavros\_msgs package
- Since 2016-02-05 (v0.17) frame conversion changed again
- Since 2016-06-22 (pre v0.18) Indigo and Jade separated from master to indigo-devel branch.
- Since 2016-06-23 (0.18.0) support MAVLink 2.0 without signing.
- Since 2017-08-23 (0.20.0) [GeographicLib][geolib] and it's datasets are required. Used to convert AMSL (FCU) and WGS84 (ROS) altitudes.
- Since 2018-05-11 (0.25.0) support building master for Indigo and Jade stopped. Mainly because update of console-bridge package.
- Since 2018-05-14 (0.25.1) support for Indigo returned. We use compatibility layer for console-bridge.
- Since 2019-01-03 (0.28.0) support for Indigo by master not guaranteed. Consider update to more recent distro.
- 2020-01-01 version 1.0.0 released, please see [#1369][iss1369] for reasons and its purpose.
- 2021-05-28 version 2.0.0 released, it's the first alpha release for ROS2.


Instructions on launching the files for navigation for the Shadow Drone:
----------------------

 - cd into workspace

 - RUN source devel/setup.bash

 - RUN sudo chmod 666 /dev/ttyPixhawk to enable serial read from Pixhawk 2.1 FCU

 - RUN roslaunch mavros shadow.launch to establish connection with Pixhawk 2.1 FCU

 - RUN rosrun mavros shadow.final (trajectory within the room) for navigation, takeoff in stabilise mode and switch to Offboard 

OR 

 - RUN rosrun mavros shadow.corner (fly forward along the corner of the room and then back but with greater altitude) for navigation, takeoff in stabilise mode and switch to Offboard

Scripts can be found under src/mavros/mavros/scripts

ADVISED TO RUN ON PYTHON 2.7.12

mavros package
--------------

It is the main package, please see its [README][mrrm].
Here you may read [installation instructions][inst].


mavros\_extras package
----------------------

This package contains some extra nodes and plugins for mavros, please see its [README][exrm].


libmavconn package
------------------

This package contain mavconn library, see its [README][libmc].
LibMAVConn may be used outside of ROS environment.


test\_mavros package
--------------------

This package contain hand-tests and [manual page][test] for APM and PX4 SITL.
Please see [README][test] first!


mavros\_msgs package
--------------------

This package contains messages and services used in MAVROS.


Support forums and chats
------------------------

Please ask your questions not related to bugs/feature or requests on:

- [MAVROS discussion in Gitter IM](https://gitter.im/mavlink/mavros)
- [PX4 Discuss Forum](https://discuss.px4.io/)
- [PX4 Slack](https://slack.px4.io/)
- [Ardupilot Discuss Forum](https://discuss.ardupilot.org/)
- [ArduPilot/VisionProjects in Gitter IM](https://gitter.im/ArduPilot/ardupilot/VisionProjects)

We'd like to keep the project bug tracker as free as possible, so please contact via the above methods. You can also PM us via Gitter and the PX4 Slack.


CI Statuses
-----------

  - ROS Melodic: [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__mavros__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__mavros__ubuntu_bionic_amd64/)
  - ROS Noetic: [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__mavros__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__mavros__ubuntu_focal_amd64/)


[mrrm]: https://github.com/mavlink/mavros/blob/master/mavros/README.md
[exrm]: https://github.com/mavlink/mavros/blob/master/mavros_extras/README.md
[libmc]: https://github.com/mavlink/mavros/blob/master/libmavconn/README.md
[test]: https://github.com/mavlink/mavros/blob/master/test_mavros/README.md
[inst]: https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation
[geolib]: https://geographiclib.sourceforge.io/
[iss1369]: https://github.com/mavlink/mavros/issues/1369
