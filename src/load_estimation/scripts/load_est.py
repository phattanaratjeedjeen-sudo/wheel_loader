#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class LoadEstimation(Node):
    def __init__(self):
        super().__init__('load_estimate')

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
        
    def sensor_filter(self):
        pass

    def convert2MPa(self):
        pass

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
