#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import random

def publisher():
    rospy.init_node('test_imu_publisher')
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.angular_velocity.z = 0.1  # Simulate slight rotation
        pub.publish(msg)
        rospy.loginfo("Published IMU data")
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
