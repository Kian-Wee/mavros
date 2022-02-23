from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ParamValue
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler




if __name__ == '__main__':
    import rostest
    rospy.init_node('teleop_node', anonymous=True)
    
    pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

### From Turtlebot to Turtledrone
# Unlike the turtlebot AGV, UGV requires more safety checks
# Pseudocode below

# Import ros binaries and messages definitions

# Initalise Node

# Ros Pub/Sub/Action/Service Initalisation
# Publish to /mavros/setpoint/local
# For debugging, /mavros/state can be suscribed to to show if it is connected and the flight mode
# 2 Services /mavros/cmd/arming & /mavros/set_mode must be initalised

# Set publishing rate for publisher

## Main Code Body
# Wait for FCU Connection
# Send a few setpoints and attempt to boot into offboard mode
# Send setpoints every few seconds while drone is in offboard mode