#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from load_estimation_interfaces.msg import Debug
from collections import deque
import numpy as np


class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')

        LgiX = 135/1000
        LgiY = 640/1000
        self.Lgi = np.sqrt(LgiY**2 + LgiX**2)
        self.Lgh = 1630/1000
        self.Lag = 3929.4/1000
        self.IGO = 180 - np.rad2deg(np.arctan2(LgiY, LgiX))

        self.theta_g_offset = -39.26
        self.theta_t_offset = 74.88

        # measured
        self.pb = None
        self.pr = None
        self.theta_g = None  
        self.theta_t = None


        # median filter
        self.window_size = 50
        self.r_history = deque(maxlen=self.window_size)
        self.b_history = deque(maxlen=self.window_size)   

        self.Ar = 0.014
        self.Ab = 0.02

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
        self.theta_g = np.rad2deg(msg.data[0]) + self.theta_g_offset
        self.theta_t = np.rad2deg(msg.data[1]) + self.theta_t_offset

    
    def pressure_callback(self, msg:Float64MultiArray):
        # convert to Pa
        self.pb = msg.data[0] * (40*10**6)/32767
        self.pr = msg.data[1] * (40*10**6)/32767

        self.b_history.append(self.pb)
        self.r_history.append(self.pr) 
        
        if len(self.b_history) < self.window_size:
            return
        
        self.pb = np.median(self.b_history)
        self.pr = np.median(self.r_history)


    def load_estimate(self):
        if self.theta_g is None or self.pb is None or self.pr is None:
            self.get_logger().warn("Load estimation called before all sensor data is available.", throttle_duration_sec=2)
            return

        Lih = np.sqrt(self.Lgh**2 + self.Lgi**2 - 2 * self.Lgh * self.Lgi * np.cos(np.deg2rad(self.theta_g + self.IGO)))
        
        asin_arg = np.clip((self.Lgh / Lih) * np.sin(np.deg2rad(self.theta_g + self.IGO)), -1.0, 1.0)
        GIH = np.rad2deg(np.arcsin(asin_arg))
        Hbmcyl = np.remainder((180 - self.IGO) - GIH,360)

        Fc = 2* (self.Ab*self.pb - self.Ar*self.pr)
        w = ((Fc * self.Lgh / self.Lag) * (np.sin(np.deg2rad(Hbmcyl)) - np.cos(np.deg2rad(Hbmcyl))*np.tan(np.deg2rad(self.theta_g)))) / 9.81
        a = (np.sin(np.deg2rad(Hbmcyl)) - np.cos(np.deg2rad(Hbmcyl)) * np.tan(np.deg2rad(self.theta_g)))
        self.publish_output(w,Fc)


    def publish_output(self,mass,force):
        msg = Debug()
        msg.f = force
        msg.w = mass
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
