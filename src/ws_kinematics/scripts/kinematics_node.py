#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

class KinematicsNode:
    def __init__(self):
        self.NUM_WHEELS = 4
        self.CMD_LEN = self.NUM_WHEELS * 2  # 4 speeds + 4 angles
        self.DIAG_LEN = 5  # Vx, Vy, omega, max_speed, scale_factor

        self.L = rospy.get_param('wheel_base_length', 0.8)
        self.W = rospy.get_param('wheel_base_width', 0.8)
        if self.L <= 0 or self.W <= 0:
            rospy.logerr(f"Invalid base dimensions L={self.L}, W={self.W}; using defaults")
            self.L = max(self.L, 0.8)
            self.W = max(self.W, 0.8)

        self.max_wheel_speed = rospy.get_param('max_wheel_speed', 1.0)
        self.min_velocity_threshold = rospy.get_param('min_velocity_threshold', 0.001)
        self.max_angle_change = rospy.get_param('max_angle_change', np.pi / 2)

        self.prev_angles = np.zeros(self.NUM_WHEELS)

        # Transformation matrix
        self.M = np.array([
            [1, 0, -self.L / 2], [0, 1,  self.W / 2],   # Front Left
            [1, 0, -self.L / 2], [0, 1, -self.W / 2],   # Front Right
            [1, 0,  self.L / 2], [0, 1,  self.W / 2],   # Rear Left
            [1, 0,  self.L / 2], [0, 1, -self.W / 2],   # Rear Right
        ])

        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.listener_callback, queue_size=10)
        self.pub = rospy.Publisher('/swerve/wheel_commands', Float64MultiArray, queue_size=10)
        self.diag_pub = rospy.Publisher('/swerve/diagnostics', Float64MultiArray, queue_size=10)

        rospy.loginfo("Kinematics Node started and listening on topic /cmd_vel")

    def normalize_angle(self, angle):
        return ((angle + np.pi) % (2 * np.pi)) - np.pi

    def optimize_wheel_direction(self, speeds, angles):
        optimized_speeds = speeds.copy()
        optimized_angles = angles.copy()

        for i in range(self.NUM_WHEELS):
            diff1 = self.normalize_angle(angles[i] - self.prev_angles[i])
            diff2 = self.normalize_angle(angles[i] + np.pi - self.prev_angles[i])
            if abs(diff2) < abs(diff1):
                optimized_speeds[i] = -speeds[i]
                optimized_angles[i] = self.normalize_angle(angles[i] + np.pi)

        self.prev_angles[:] = optimized_angles
        return optimized_speeds, optimized_angles

    def listener_callback(self, msg):
        try:
            rospy.loginfo(f"Received /cmd_vel: Vx={msg.linear.x:.2f}, Vy={msg.linear.y:.2f}, omega={msg.angular.z:.2f}")

            Vx, Vy, omega = msg.linear.x, msg.linear.y, msg.angular.z
            if (abs(Vx) < self.min_velocity_threshold and
                abs(Vy) < self.min_velocity_threshold and
                abs(omega) < self.min_velocity_threshold):
                out = Float64MultiArray()
                out.layout.dim = [MultiArrayDimension(label="data", size=self.CMD_LEN, stride=self.CMD_LEN)]
                out.data = np.concatenate((np.zeros(self.NUM_WHEELS), self.prev_angles)).tolist()
                self.pub.publish(out)
                return

            Vc = np.array([Vx, Vy, omega])
            Vw = self.M.dot(Vc)
            speeds = np.hypot(Vw[0::2], Vw[1::2])
            angles = np.arctan2(Vw[1::2], Vw[0::2])

            speeds, angles = self.optimize_wheel_direction(speeds, angles)

            scale_factor = 1.0
            max_speed = np.max(speeds)
            if max_speed > self.max_wheel_speed:
                scale_factor = self.max_wheel_speed / max_speed
                speeds *= scale_factor
                rospy.logdebug(f"Scaling wheel speeds by factor {scale_factor:.3f}")

            out = Float64MultiArray()
            out.layout.dim = [MultiArrayDimension(label="data", size=self.CMD_LEN, stride=self.CMD_LEN)]
            out.data = np.concatenate((speeds, angles)).tolist()
            self.pub.publish(out)

            if self.diag_pub.get_num_connections() > 0:
                diag = Float64MultiArray()
                diag.layout.dim = [MultiArrayDimension(label="data", size=self.DIAG_LEN, stride=self.DIAG_LEN)]
                diag.data = [Vx, Vy, omega, max_speed, scale_factor]
                self.diag_pub.publish(diag)

        except Exception as e:
            rospy.logerr(f"Error in listener_callback: {e}")

    def shutdown(self):
        zero = Float64MultiArray()
        zero.layout.dim = [MultiArrayDimension(label="data", size=self.CMD_LEN, stride=self.CMD_LEN)]
        zero.data = [0.0] * self.CMD_LEN
        self.pub.publish(zero)
        rospy.loginfo('Kinematics Node shutting down - wheels set to zero')


def main():
    rospy.init_node('kinematics')
    node = KinematicsNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
