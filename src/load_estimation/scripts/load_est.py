#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from collections import deque
import numpy as np


class LoadEstimation(Node):
    def __init__(self):
        super().__init__('load_estimate')

        self.window_size = 30
        self.r_history = deque(maxlen=self.window_size)
        self.b_history = deque(maxlen=self.window_size)

        self.create_service(Trigger, 
            "/vehicle/load_estimate", 
            self.load_estimate_callback
        )

        self.create_subscription(
            Float64MultiArray, 
            "/vehicle/loader_joint_pressures", 
            self.pressure_callback, 
            10
        )

    def pressure_callback(self, msg:Float64MultiArray):
        raw_b = msg.data[0]
        raw_r = msg.data[1]

        # median filter
        self.b_history.append(raw_b)
        self.r_history.append(raw_r)
        median_b = np.median(self.b_history)
        median_r = np.median(self.r_history)

        if len(self.b_history) < self.window_size:
            return

        # offset cal
        load_Pb = 0
        load_Pr = 0

        # convert2bar
        Pb = 400/32767 * load_Pb
        Pr = 400/32767 * load_Pr

    def geometry_cal(self):
        pass

    def load_cal(self):
        pass
    
    def load_estimate_callback(self, request, response):
        self.get_logger().info('Load estimation initialized.')
        response.success = True
        response.message = 'Load calculation triggered successfully.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LoadEstimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
