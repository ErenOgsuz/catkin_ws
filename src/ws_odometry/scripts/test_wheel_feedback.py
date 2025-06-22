#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import math

def publisher():
    rospy.init_node('test_wheel_feedback_publisher')
    pub = rospy.Publisher('/wheel_feedback', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        # Format: [v1, v2, v3, v4, a1, a2, a3, a4]
        # All wheels moving forward at 1 m/s, all steering straight
        msg.data = [1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(msg)
        rospy.loginfo("Published wheel feedback")
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
