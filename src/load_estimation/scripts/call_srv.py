#!/usr/bin/python3

# from load_estimation.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')

        self.create_subscription(
            Float64MultiArray,
            '/vehicle/loader_joint_angles',
            self.angle_callback,
            10
        )

def angle_callback(self, msg:Float64MultiArray):
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.theta_g = msg.data[0] + self.theta_g_offset
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

    

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
