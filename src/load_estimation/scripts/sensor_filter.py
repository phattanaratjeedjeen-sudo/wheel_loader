#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque

class SensorFilter(Node):
    def __init__(self):
        super().__init__('sensor_filter')

        # --- Declare and Get ROS2 Parameters ---
        self.declare_parameter('alpha', 0.1)
        self.declare_parameter('window_size', 50)

        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value

        self.get_logger().info(f"Using low-pass filter alpha: {self.alpha}")
        self.get_logger().info(f"Using median filter window size: {self.window_size}")

        # --- Filter State Variables ---
        self.lowpass_b = None
        self.lowpass_r = None
        self.b_history = deque(maxlen=self.window_size)
        self.r_history = deque(maxlen=self.window_size)

        # --- Subscribers and Publishers ---
        self.pressure_sub = self.create_subscription(
            Float64MultiArray,
            '/vehicle/loader_joint_pressures',
            self.pressure_callback,
            10
        )

        self.lowpass_pub = self.create_publisher(
            Float64MultiArray,
            '/vehicle/lowpass_pressures',
            10
        )

        self.median_pub = self.create_publisher(
            Float64MultiArray,
            '/vehicle/median_pressures',
            10
        )


    def pressure_callback(self, msg:Float64MultiArray):
        raw_b = msg.data[0]
        raw_r = msg.data[1]
        self.lowpass_filter(raw_b, raw_r)
        self.median_filter(raw_b, raw_r)


    def lowpass_filter(self, raw_b, raw_r):
        if self.lowpass_b is None or self.lowpass_r is None:
            self.lowpass_b = raw_b
            self.lowpass_r = raw_r

        self.lowpass_b = self.alpha * raw_b + (1 - self.alpha) * self.lowpass_b
        self.lowpass_r = self.alpha * raw_r + (1 - self.alpha) * self.lowpass_r

        msg = Float64MultiArray()
        msg.data = [self.lowpass_b, self.lowpass_r]
        self.lowpass_pub.publish(msg)


    def median_filter(self, raw_b, raw_r):
        self.b_history.append(raw_b)
        self.r_history.append(raw_r)

        if len(self.b_history) < self.window_size:
            return

        median_b = np.median(self.b_history)
        median_r = np.median(self.r_history)

        msg = Float64MultiArray()
        msg.data = [median_b, median_r]
        self.median_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
