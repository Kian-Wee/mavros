#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header, Float64 #type 

def blast3dk(xx, yy, zz, yaw):
    pub = rospy.Publisher('pose3dk', Vector3, queue_size=10)
    yaw_pub = rospy.Publisher('yaw3dk', Float64, queue_size=10)
    rospy.init_node('pose3dk_yaw3dk', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #ros_3dk = PoseStamped()
        #ros_3dk.header = Header()
        #ros_3dk.header.stamp = rospyQ.Time.now()
        ros_3dk = Vector3()
        yaw_3dk = Float64()
        yaw_3dk.data = yaw
        ros_3dk.x = xx
        ros_3dk.y = yy
        ros_3dk.z = zz
        rospy.loginfo(ros_3dk)
        rospy.loginfo(yaw_3dk.data)
        yaw_pub.publish(yaw_3dk)
        pub.publish(ros_3dk)
        rate.sleep()


if __name__ == '__main__':
    try:
        x = float(raw_input('x: '))
        y = float(raw_input('y: '))
        z = float(raw_input('z: '))
        yaw = float(raw_input('yaw: '))
        blast3dk(x, y, z, yaw)
    except rospy.ROSInterruptException:
        pass
