#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque


class LoadEstimationNode(Node):
    def __init__(self):
        super().__init__('load_estimation_node')

        # measured
        self.theta_g = None      # boom angle (rad)
        self.pb = None           # bottom cylinder pressure (Pa)
        self.pr = None           # rod side pressure (Pa)
        
        # constant
        self.theta_g_offset = -0.633    # boom angle offset (rad)
        self.ab = 0.02                  # bottom cylinder area (m^2)
        self.ar = 0.014                 # rod side area (m^2)
        self.toPa = 40*10**6/32767      # convert sensor data to Pa

        # link geometry 
        LgiX = 135/1000
        LgiY = 640/1000
        self.Lgh = 1630/1000
        self.Lag = 3929.4/1000
        self.Lgi = np.sqrt(LgiY**2 + LgiX**2)
        self.IGO = np.pi - (np.arctan2(LgiY, LgiX))

        # median filter
        self.window_size = 50
        self.r_history = deque(maxlen=self.window_size)
        self.b_history = deque(maxlen=self.window_size)
        
        self.create_service(
            Trigger,
            '/vehicle/load_estimate',
            self.srv_callback
        )

        self.create_subscription(
            Float64MultiArray,
            '/vehicle/loader_joint_pressures',
            self.pressure_callback,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/vehicle/loader_joint_angles',
            self.angle_callback,
            10
        )


    def angle_callback(self, msg:Float64MultiArray):
        self.theta_g = msg.data[0] + self.theta_g_offset


    def pressure_callback(self, msg:Float64MultiArray):
        pb_raw = msg.data[0]
        pr_raw = msg.data[1]

        self.b_history.append(pb_raw)
        self.r_history.append(pr_raw) 
        
        if len(self.b_history) < self.window_size:
            self.pb = None # Not ready
            self.pr = None
            return
        
        self.pb = np.median(list(self.b_history)) * self.toPa
        self.pr = np.median(list(self.r_history)) * self.toPa

    def calculate_load_mass(self):
        # geometry
        Lih = np.sqrt(self.Lgh**2 + self.Lgi**2 - 2 * self.Lgh * self.Lgi * np.cos(self.theta_g + self.IGO))
        GIH = np.arcsin(np.clip((self.Lgh / Lih) * np.sin(self.theta_g + self.IGO), -1.0, 1.0))
        Hbmcyl = np.pi - self.IGO - GIH
        a = np.sin(Hbmcyl) - np.cos(Hbmcyl) * np.tan(self.theta_g)

        # cylinder force (N)
        Fc = 2 * (self.ab * self.pb - self.ar * self.pr)
        
        simple_lever = ((Fc * self.Lgh / self.Lag) * min(0.40,a))/9.807
        link_compensate = 3640 * np.cos(self.theta_g + 0.225) / np.cos(self.theta_g)
        low_compensate = min(20.0,(np.tan(self.theta_g - 0.29) / np.cos(self.theta_g)) * 1200 + 1000)

        # estimated mass (kg)
        w = max(0.0,simple_lever - link_compensate - low_compensate)
        return w


    def srv_callback(self, request, response):
        try:
            if self.theta_g is None or self.pb is None or self.pr is None:
                raise Exception("Sensor data not ready.")
            load_mass = self.calculate_load_mass()
            response.success = True
            response.message = f'load(kg): {load_mass:.0f}' 
        except Exception as e:
            response.success = False
            response.message = f'{str(e)}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LoadEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()