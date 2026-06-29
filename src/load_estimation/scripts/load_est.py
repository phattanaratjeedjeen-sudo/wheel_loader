#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import asyncio
from collections import deque
import numpy as np


class LoadEstimation(Node):
    def __init__(self):
        super().__init__('load_estimate')

        # linkage length (m)
        self.Ldg = 2494.4/1000
        self.Lfg = 356.9/1000
        self.LfgX = 350/1000
        self.LfgY = 70/1000
        self.Lag = 3929.4/1000
        self.Lde = 926.3/1000
        self.Lad = 1689.5/1000
        self.Lcd = 1133.2/1000
        self.Lbc = 984.4/1000
        self.Lab = 454.8/1000
        self.LgiX = -13.5*10/1000
        self.LgiY = -64*10/1000
        self.Lgi = np.sqrt(self.LgiY**2 + self.LgiX**2)/1000
        self.Lgh = 1630/1000

        self.n = 2.0 

        # angle (rad)
        self.DGA = np.deg2rad(16.38)
        self.FGO = np.arctan(self.LfgY/self.LfgX)
        self.theta_d = np.deg2rad(31.45)
        self.DEC = np.deg2rad(17.35)
        self.IGO = np.pi - np.arctan(self.LgiY/self.LgiX)

        # area (m^2)
        self.Ab = 0.02;     # bottom cross-section area
        self.Ar = 0.014;    # rod cross-section area

        # measured
        self.theta_g = None # AGO (deg)
        self.theta_t = None # FEC (deg)
        self.Pb = None      # bottom cylinder pressure (Pa)
        self.Pr = None      # rod side pressure (Pa)

        # filter
        self.window_size = 50
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

        self.create_subscription(
            Float64MultiArray, 
            "/vehicle/loader_joint_angles", 
            self.angle_callback,
            10
        )

    def pressure_callback(self, msg:Float64MultiArray):
        raw_b = msg.data[0]
        raw_r = msg.data[1]

        # median filter
        self.b_history.append(raw_b)
        self.r_history.append(raw_r)

        if len(self.b_history) < self.window_size:
            return

        median_b = np.median(self.b_history)
        median_r = np.median(self.r_history)

        # offset cal
        offset_b = 3874 + 2727*median_b + -912*median_b**2
        offset_r = 36.5*median_r  + 394

        # sensor_val2Pa, pure_load pressure
        self.Pb = (median_b - offset_b) * (40*10**6)/32767
        self.Pr = (median_r - offset_r) * (40*10**6)/32767

    def angle_callback(self, msg:Float64MultiArray):
        self.theta_g = msg.data[0]
        self.theta_t = msg.data[1]

    def load_estimate(self):
        if self.theta_g is None or self.theta_t is None or self.Pb is None or self.Pr is None:
            self.get_logger().warn("Load estimation called before all sensor data is available.", throttle_duration_sec=1)
            return None

        try:
            # eq 15
            FGD = self.FGO + self.theta_g + self.DGA
            Ldf = np.sqrt(self.Lfg**2 + self.Ldg**2 - 2*self.Lfg*self.Ldg*np.cos(FGD))

            # eq 16
            Laf = np.sqrt(self.Lag**2 + self.Lfg**2 - 2*self.Lag*self.Lfg*np.cos(self.theta_g - self.FGO))

            # add
            DEF = self.theta_t + self.DEC
            EFD = np.arcsin(self.Lde * np.sin(DEF) / Ldf)
            EDF = np.pi - EFD - DEF

            # tilt prismatic length
            Lef = self.Lde * np.sin(EDF) / np.sin(EFD)

            # eq 17
            a3 = self.Ldg * np.sin(self.theta_g + self.DGA) - self.LfgY
            a4 = self.Ldg * np.cos(self.theta_g + self.DGA) - self.LfgX
            a5 = Ldf**2 + Lef**2 - self.Lde**2
            a6 = 2 * Ldf * Lef
            theta_f = np.arctan2(a3, a4) + np.arccos(a5/a6)

            # eq 18
            theta_e = -(np.pi - self.DEC - self.theta_t)

            # eq 19
            a9 = self.Lad**2 + self.Ldg**2 - self.Lag**2
            a10 = 2 * self.Lad * self.Ldg
            ADC = np.arccos(a9/a10) + EDF - np.pi - self.theta_d

            # eq 20
            Lac = np.sqrt(self.Lad**2 + self.Lcd**2 - 2*self.Lad*self.Lcd*np.cos(ADC))

            # eq 21
            a13 = Lac**2 + self.Lbc**2 - self.Lab**2
            a14 = 2 * Lac * self.Lbc
            a15 = self.Lcd**2 + Lac**2 - self.Lad**2
            a16 = 2 * self.Lcd * Lac
            theta_c = np.pi + np.arccos(a13/a14) - np.arccos(a15/a16)

            # eq 22
            a17 = self.Lab**2 + self.Lbc**2 - Lac**2
            a18 = 2 * self.Lab * self.Lbc
            theta_b = np.arccos(a17/a18) - np.pi

            # lift prismatic length
            Lih = np.sqrt(self.Lgh**2 + self.Lgi**2 - 2*self.Lgh*self.Lgi*np.cos(self.theta_g + self.IGO))

            GIH = np.arcsin((self.Lgh/Lih) * np.sin(self.theta_g + self.IGO))

            # eq 14
            Hbmcyl = self.IGO - GIH
            theta_i = np.pi - (self.IGO + Hbmcyl)

            a = 0
            # eq 8
            b = -self.Lab * np.sin(theta_b)
            # eq 9 
            c = self.Lcd * np.sin(theta_c)
            # eq 10
            d = -self.Lde * np.sin(theta_e)
            # eq 11
            e = self.Lfg * np.sin(np.pi - theta_f + self.FGO)
            # eq 12
            f = self.Lgi * np.sin(theta_i)
            # eq 13
            Lw = self.Lag * np.cos(self.theta_g) + a
            # force applied to lift arm cylinders
            Fc = self.n * (self.Ab * self.Pb - self.Ar * self.Pr)  
            W = Fc*f / (Lw - (a*c*e/b*d))
            return W / 9.81
        
        except :
            return None
        

    async def load_estimate_callback(self, request, response):
        self.get_logger().info('Load estimation triggered. Wait 0.5sec to estimate load')
        self.r_history.clear()
        self.b_history.clear()
        await asyncio.sleep(0.5)
        try:
            estimated_mass = self.load_estimate()
            if estimated_mass is not None:
                response.success = True
                response.message = f'Estimated load mass: {estimated_mass:.2f} kg'
            else:
                response.success = False
                response.message = 'Load estimation failed. Not all sensor data is available or calculation error.'
        except Exception as e:
            response.success = False
            response.message = f'Exception during load estimation: {str(e)}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LoadEstimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
