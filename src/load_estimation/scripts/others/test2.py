#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from load_estimation_interfaces.msg import Debug
import numpy as np


class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')

        self.gh = 1.63          # distance b/w G-H (m)
        self.gi = 0.654         # distance b/w G-I (m)
        self.ga = 3.93          # distance b/w G-A (m)
        self.igo = 1.779        # angle from horizontal line - I intersec at G (rad)
        self.gio = 1.377        # angle from horizontal line - G intersec at I (rad)
        self.area_r = 0.014     # piston side area (m^2)
        self.area_b = 0.02      # bottom cylinder area (m^2)
        self.n = 2              # number of lift cylinders

        self.theta_g_offset = -0.6852

        # measured
        self.pb = None
        self.pr = None
        self.theta_g = None    

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
        self.theta_g = msg.data[0] + self.theta_g_offset
        

    def pressure_callback(self, msg:Float64MultiArray):
        # convert to Pa
        self.pb = msg.data[0] * (40*10**6)/32767
        self.pr = msg.data[1] * (40*10**6)/32767

    def median_filter(self):
        pass

    def offset_cal(self):
        pass

    def load_estimate(self):
        if self.theta_g is None or self.pb is None or self.pr is None:
            return

        ih = np.sqrt(self.gh**2 + self.gi**2 - 2*self.gh*self.gi*np.cos(self.theta_g + self.igo))
        gih = np.arcsin((self.gh/ih) * np.sin(self.theta_g + self.igo))
        oih = self.gio - gih

        f = self.n * (self.area_b * self.pb - self.area_r * self.pr)
        w = f * (self.gh/self.ga) * (np.sin(oih) - np.cos(oih)*np.tan(self.theta_g))
        self.publish_output(w/9.81,self.theta_g,self.pb,self.pr,f,ih,gih,oih)


    def publish_output(self,mass,theta_g,pb,pr,f,ih,gih,oih):
        msg = Debug()
        msg.f = f
        msg.w = mass
        msg.theta_g = theta_g
        msg.pb = pb
        msg.pr = pr
        msg.ih = ih
        msg.gih = gih
        msg.oih = oih
        self.output_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
