#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray  
from rcl_interfaces.msg import ParameterDescriptor    
from collections import deque
from load_estimation_interfaces.msg import Debug
import numpy as np

class LoadEstimation(Node):
    def __init__(self):
        super().__init__('load_estimate')

        self.declare_parameter('use_filter', True)
        self.declare_parameter('use_offset', True)
        self.use_filter = self.get_parameter('use_filter').get_parameter_value().bool_value
        self.use_offset = self.get_parameter('use_offset').get_parameter_value().bool_value

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
        self.Lgi = np.sqrt(self.LgiY**2 + self.LgiX**2)
        self.Lgh = 1630/1000

        # number of lift cylinders
        self.n = 2.0 

        # angle (rad)
        self.DGA = np.deg2rad(16.38)
        self.FGO = np.arctan(self.LfgY/self.LfgX)
        self.theta_d = np.deg2rad(31.45)
        self.DEC = np.deg2rad(17.35)
        self.IGO = np.pi - np.arctan(self.LgiY/self.LgiX)

        # area (m^2)
        self.Ab = 0.02;     # bottom cross-section area
        self.Ar = 0.014;    # piston cross-section area

        # measured
        self.theta_g = 0.0    # AGO (rad)
        self.theta_t = 0.0   # FEC (rad)
        self.Pb = None       # bottom cylinder pressure (Pa)
        self.Pr = None       # rod side pressure (Pa)
        self.raw_Pb = 0.0
        self.raw_Pr = 0.0
        self.offset_Pb = 0.0
        self.offset_Pr = 0.0
        self.median_Pb = None
        self.median_Pr = None

        self.theta_g_offset = np.deg2rad(-39.26)
        self.theta_t_offset = np.deg2rad(74.88)

        # filter
        self.window_size = 50
        self.r_history = deque(maxlen=self.window_size)
        self.b_history = deque(maxlen=self.window_size)

        hz = 100
        self.create_timer(1/hz, self.load_estimate)

        self.create_subscription(
            Float64MultiArray, 
            "/vehicle/loader_joint_pressures", 
            self.pressure_callback, 
            10,
        )

        self.create_subscription(
            Float64MultiArray, 
            "/vehicle/loader_joint_angles", 
            self.angle_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Debug, 
            "/debug", 
            10
        )

    def pressure_callback(self, msg:Float64MultiArray):
        self.raw_b = msg.data[0]
        self.raw_r = msg.data[1]

        if self.use_filter:
            # median filter
            self.b_history.append(self.raw_b)
            self.r_history.append(self.raw_r)

            if len(self.b_history) < self.window_size:
                return

            self.median_Pb = np.median(self.b_history)
            self.median_Pr = np.median(self.r_history)
            Pb = self.median_Pb
            Pr = self.median_Pr

        else:
            Pb = self.raw_b
            Pr = self.raw_r

        # offset cal
        if self.use_offset:
            self.offset_Pb = 4225 + 1979*self.theta_g + -534*self.theta_g**2
            self.offset_Pr = 350

        self.Pb = np.maximum(0.0,(Pb - self.offset_Pb)) * (40*10**6)/32767
        self.Pr = np.maximum(0.0,(Pr - self.offset_Pr)) * (40*10**6)/32767

    def angle_callback(self, msg:Float64MultiArray):
        self.theta_g = msg.data[0]
        self.theta_t = msg.data[1]


    def load_estimate(self):
        if self.theta_g is None or self.theta_t is None or self.Pb is None or self.Pr is None:
            return
        
        theta_g = self.theta_g + self.theta_g_offset
        Lih = np.sqrt(self.Lgh**2 + self.Lgi**2 - 2*self.Lgh*self.Lgi*np.cos(theta_g + self.IGO))  
        GIH = np.arcsin((self.Lgh/Lih) * np.sin(theta_g + self.IGO))
        Hbmcyl = np.pi - self.IGO - GIH
        theta_i = np.pi - (self.IGO + Hbmcyl)
        f = self.Lgi * np.sin(theta_i)
        Lw = self.Lag * np.cos(theta_g) 
        Fc = self.n * (self.Ab * self.Pb - self.Ar * self.Pr)  
        W = (Fc*f/Lw)/9.81

        msg = Debug()
        msg.theta_i = theta_i
        msg.theta_g = self.theta_g
        msg.pb = self.Pb
        msg.pr = self.Pr
        msg.f = f
        msg.lw = Lw
        msg.fc = Fc
        msg.w = W
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LoadEstimation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
