#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion
import numpy as np
import math

class OdometryNode():
    def __init__(self):  # NOTE: use __init, not _init
        rospy.init_node('odometry_node', anonymous=True)

        # Declare parameters with default values
        self.L = rospy.get_param('~wheel_base_length', 0.79)
        self.W = rospy.get_param('~wheel_base_width', 0.79)
        self.imu_weight = rospy.get_param('~imu_weight', 0.8)


        # Transformation matrix from individual wheel velocities to chassis velocity
        self.M = np.array([
            [1, 0, -self.L / 2], [0, 1, self.W / 2],   # Front Left
            [1, 0, -self.L / 2], [0, 1, -self.W / 2],  # Front Right
            [1, 0, self.L / 2],  [0, 1, self.W / 2],   # Back Left
            [1, 0, self.L / 2],  [0, 1, -self.W / 2],  # Back Right
        ])

        # Publishers and Subscribers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.wheel_sub = rospy.Subscriber('wheel_feedback', Float64MultiArray, self.wheel_callback)
        self.imu_sub = rospy.Subscriber('imu/data', Imu, self.imu_callback)


        # Robot state
        self.x = 0.0         # Position X
        self.y = 0.0         # Position Y
        self.theta = 0.0     # Orientation (yaw)

        self.latest_imu = None  # Latest IMU data
        self.last_time = rospy.Time.now()  # Timestamp of last update

        rospy.loginfo("Odometry Node started.")

    def imu_callback(self, msg):
        """
        Store the latest IMU message for later fusion with wheel data.
        """
        self.latest_imu = msg

    def wheel_callback(self, msg):
        """
        Process wheel speeds and angles to compute odometry.
        """
        data = msg.data
        if len(data) < 8:
            rospy.logwarn("Wheel feedback message does not contain a complete data package.")
            return

        speeds = data[:4]  # Linear speed for each wheel (m/s)
        angles = data[4:8] # Steering angle for each wheel (radians)

        # Convert wheel speeds and angles to velocity components in X and Y
        V_wheels = []
        for i in range(4):
            v = speeds[i]
            a = angles[i]
            V_wheels.append(v * math.cos(a))  # X component
            V_wheels.append(v * math.sin(a))  # Y component
        V_wheels = np.array(V_wheels)

        # Calculate chassis velocity using pseudo-inverse of the matrix
        M_pinv = np.linalg.pinv(self.M)
        V_chassis = M_pinv @ V_wheels
        V_x = V_chassis[0]
        V_y = V_chassis[1]
        wheel_omega = V_chassis[2]  # Angular velocity from wheels

        # Angular velocity fusion with IMU
        if self.latest_imu is not None:
            imu_omega = self.latest_imu.angular_velocity.z
            omega = self.imu_weight * imu_omega + (1 - self.imu_weight) * wheel_omega
        else:
            omega = wheel_omega

        # Time delta calculation
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Euler Integration
        # Update heading
        self.theta += omega * dt

        # Transform to global frame
        global_Vx = V_x * math.cos(self.theta) - V_y * math.sin(self.theta)
        global_Vy = V_x * math.sin(self.theta) + V_y * math.cos(self.theta)

        # Integrate global position
        self.x += global_Vx * dt
        self.y += global_Vy * dt

        # Create and publish the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.yaw_to_quaternion(self.theta)

        odom_msg.twist.twist.linear.x = V_x
        odom_msg.twist.twist.linear.y = V_y
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

        # Logging for debugging
        rospy.loginfo(f"Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
        rospy.loginfo(f"Chassis Velocities: Vx={V_x:.2f}, Vy={V_y:.2f}, omega={omega:.2f}")

    def yaw_to_quaternion(self, yaw):
        """
        Convert a yaw angle (in radians) into a Quaternion message.
        """
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q
    
    def spin(self):
        rospy.spin()

def main(args=None):
    try:
        node = OdometryNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

# Ensure script is run directly
if __name__ == '__main__':
    main()
