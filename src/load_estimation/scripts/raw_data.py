#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from load_estimation_interfaces.msg import Debug

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        # measured
        self.pb = None
        self.pr = None
        self.theta_g = None  
        self.theta_t = None

        hz = 100
        self.create_timer(1/hz, self.load_estimate)

        self.create_subscription(
            Float64MultiArray,
            "/vehicle/loader_joint_pressures",
            self.pressure_callback,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            "/vehicle/loader_joint_angles",
            self.angle_callback,
            10
        )

        self.output_pub = self.create_publisher(
            Debug,
            "/debug",
            10
        )

    def angle_callback(self, msg:Float64MultiArray):
        self.theta_g = msg.data[0]
        self.theta_t = msg.data[1]

    def pressure_callback(self, msg:Float64MultiArray):
        # convert to Pa
        self.pb = msg.data[0]
        self.pr = msg.data[1]

    def load_estimate(self):
        if self.theta_g is None or self.pb is None or self.pr is None or self.theta_t is None:
            self.get_logger().warn("Sensor data or filter not ready.", throttle_duration_sec=2)
            return
        self.publish_output()

    def publish_output(self):
        msg = Debug()
        msg.theta_g = self.theta_g
        msg.theta_t = self.theta_t
        msg.pb = self.pb
        msg.pr = self.pr
        self.output_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()