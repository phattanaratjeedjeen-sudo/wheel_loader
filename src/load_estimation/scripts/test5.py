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
        self.IGO = np.pi - (np.arctan2(LgiY, LgiX))

        self.theta_g_offset = -0.633 # rad
        self.theta_t_offset = 1.307  # rad
 
        # measured
        self.pb = None
        self.pr = None
        self.theta_g = None  
        self.theta_t = None

        # median filter
        self.window_size = 50
        self.r_history = deque(maxlen=self.window_size)
        self.b_history = deque(maxlen=self.window_size)

        # area
        self.Ar = 0.014
        self.Ab = 0.02

        # link mass compensate
        self.k = 3640
        self.beta = 0.225

        hz = 10
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
        self.theta_g = msg.data[0] + self.theta_g_offset
        self.theta_t = msg.data[1] + self.theta_t_offset

    
    def pressure_callback(self, msg:Float64MultiArray):
        # convert to Pa
        pb = msg.data[0] * (40*10**6)/32767
        pr = msg.data[1] * (40*10**6)/32767

        self.b_history.append(pb)
        self.r_history.append(pr) 
        
        if len(self.b_history) < self.window_size:
            return
        
        self.pb = np.median(list(self.b_history))
        self.pr = np.median(list(self.r_history))


    def load_estimate(self):

        if self.theta_g is None or self.pb is None or self.pr is None:
            self.get_logger().warn("Sensor data or filter not ready.", throttle_duration_sec=2)
            return
        
        # geometry
        Lih = np.sqrt(self.Lgh**2 + self.Lgi**2 - 2 * self.Lgh * self.Lgi * np.cos(self.theta_g + self.IGO))
        GIH = np.arcsin(np.clip((self.Lgh / Lih) * np.sin(self.theta_g + self.IGO), -1.0, 1.0))
        Hbmcyl = np.pi - self.IGO - GIH
        a = np.sin(Hbmcyl) - np.cos(Hbmcyl) * np.tan(self.theta_g)

        # cylinder force (N)
        Fc = 2* (self.Ab*self.pb - self.Ar*self.pr)
        # simple lever
        simple_lever = ((Fc * self.Lgh / self.Lag) * min(0.40,a))/9.807
        # arm mass compensate 
        link_compensate = self.k * np.cos(self.theta_g + self.beta) / np.cos(self.theta_g)
        # low angle compensate
        low_compensate = min(20.0,(np.tan(self.theta_g - 0.29) / np.cos(self.theta_g)) * 1200 + 1000)
        # load mass (kg)
        w = simple_lever - link_compensate - low_compensate
        
        self.publish_output(w,Fc,link_compensate,simple_lever,low_compensate,a)


    def publish_output(self,mass,force,arm,main,lowang,geometry):
        msg = Debug()
        msg.f = force
        msg.w = mass
        msg.theta_g = self.theta_g
        msg.theta_t = self.theta_t
        msg.pb = self.pb
        msg.pr = self.pr
        msg.arm = arm
        msg.main = main
        msg.lowang = lowang
        msg.a = geometry
        self.output_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()