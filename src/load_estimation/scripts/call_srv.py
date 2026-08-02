#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from collections import deque

class ServiceCallerNode(Node):
    def __init__(self):
        super().__init__('load_estimate_caller_node')

        self.vel_threshold = 0.003
        self.acc_threshold = 0.005
        self.theta_g_offset = -0.62

        # EMA filter for theta_g
        self.theta_g_ema_alpha = 0.1
        self.theta_g_ema_filtered = None

        # State variables
        self.theta_g = None
        self.vel_g = None
        self.acc_g = None
        self.is_moving = True
        self.service_call_in_progress = False

        # For derivatives
        self.theta_g_history = deque(maxlen=3)
        self.vel_g_history = deque(maxlen=3)

        self.create_subscription(
            Float64MultiArray,
            '/vehicle/loader_joint_angles',
            self.angle_callback,
            10
        )

        self.client = self.create_client(Trigger, '/load_estimate')

    def angle_callback(self, msg: Float64MultiArray):
        current_time = self.get_clock().now().nanoseconds / 1e9
        raw_theta_g = msg.data[0] + self.theta_g_offset

        # EMA filter for theta_g
        if self.theta_g_ema_filtered is None:
            self.theta_g_ema_filtered = raw_theta_g
        else:
            self.theta_g_ema_filtered = self.theta_g_ema_alpha * raw_theta_g + (1 - self.theta_g_ema_alpha) * self.theta_g_ema_filtered

        self.theta_g = self.theta_g_ema_filtered
        self.theta_g_history.append((current_time, self.theta_g))

        # Calculate velocity using central difference
        if len(self.theta_g_history) == 3:
            t0, y0 = self.theta_g_history[0]
            t1, _ = self.theta_g_history[1]
            t2, y2 = self.theta_g_history[2]

            dt = t2 - t0
            if dt > 1e-9:
                # Central difference for derivative at t1
                self.vel_g = (y2 - y0) / dt
                self.vel_g_history.append((t1, self.vel_g))
            else:
                self.vel_g = 0.0
        else:
            self.vel_g = None

        # Calculate acceleration using central difference
        if len(self.vel_g_history) == 3:
            t0, y0 = self.vel_g_history[0]
            t2, y2 = self.vel_g_history[2]

            dt = t2 - t0
            if dt > 1e-9:
                self.acc_g = (y2 - y0) / dt
            else:
                self.acc_g = 0.0
        else:
            self.acc_g = None

        if self.vel_g is None or self.acc_g is None:
            return

        currently_static = abs(self.vel_g) < self.vel_threshold and abs(self.acc_g) < self.acc_threshold

        if currently_static:
            if self.is_moving and not self.service_call_in_progress:
                self.get_logger().info(f"static met: vel={self.vel_g:.4f}, acc={self.acc_g:.4f}). Call service")
                self.call_load_estimate_service()
                self.is_moving = False
        else:  # is moving
            if not self.is_moving:
                self.is_moving = True

    def call_load_estimate_service(self):
        if self.service_call_in_progress:
            # self.get_logger().warn("Service call is already in progress. Skipping new request.", throttle_duration_sec=5)
            return

        if not self.client.service_is_ready():
            # self.get_logger().warn("Service /load_estimate not available, skipping call.", throttle_duration_sec=5)
            return

        self.service_call_in_progress = True
        self.get_logger().info("Requesting load estimation...")
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service call successful: {response.message}")
            else:
                self.get_logger().warn(f"Service call failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed with exception: {e}")
        finally:
            self.service_call_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = ServiceCallerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
