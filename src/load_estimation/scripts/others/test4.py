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

        # states
        self.omega_threshold = 0.4  # deg/s
        self.alpha_threshold = 50.0 # deg/s^2
        self.theta_g_old = None
        self.v_old = None
        self.last_state_cal_time = None
        self.omega = 0.0
        self.alpha = 0.0

        # area
        self.Ar = 0.014
        self.Ab = 0.02

        # link mass compensate
        self.k = 3650
        self.beta = 13

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

    
    def states_cal(self):
        if self.theta_g is None:
            return False 

        current_time = self.get_clock().now()

        if self.theta_g_old is None or self.last_state_cal_time is None:
            self.theta_g_old = self.theta_g
            self.last_state_cal_time = current_time
            return False 

        dt = (current_time - self.last_state_cal_time).nanoseconds / 1e9
        
        if dt < 1e-6:
            return True 

        v = (self.theta_g - self.theta_g_old) / dt
        
        if self.v_old is None:
            self.v_old = v
            self.omega = v
            self.theta_g_old = self.theta_g
            self.last_state_cal_time = current_time
            return False 

        a = (v - self.v_old) / dt

        self.omega = v
        self.alpha = a
        self.v_old = v
        self.theta_g_old = self.theta_g
        self.last_state_cal_time = current_time
        return True 

    
    def pressure_callback(self, msg:Float64MultiArray):
        # convert to Pa
        pb = msg.data[0] * (40*10**6)/32767
        pr = msg.data[1] * (40*10**6)/32767

        self.b_history.append(pb)
        self.r_history.append(pr) 
        
        if len(self.b_history) < self.window_size:
            return
        
        self.pb = np.median(self.b_history)
        self.pr = np.median(self.r_history)


    def load_estimate(self):
        # is_state_ready = self.states_cal()

        if self.theta_g is None or self.pb is None or self.pr is None:
            self.get_logger().warn("Sensor data or filter not ready, skipping estimation.", throttle_duration_sec=2)
            return
        
        # if not is_state_ready:
        #     self.get_logger().warn("State estimation (v, a) is initializing.", throttle_duration_sec=5)
        #     return
        
        # if abs(self.alpha) < self.alpha_threshold and abs(self.omega) < self.omega_threshold:
        
        # geometry
        Lih = np.sqrt(self.Lgh**2 + self.Lgi**2 - 2 * self.Lgh * self.Lgi * np.cos(np.deg2rad(self.theta_g + self.IGO)))
        asin_arg = np.clip((self.Lgh / Lih) * np.sin(np.deg2rad(self.theta_g + self.IGO)), -1.0, 1.0)
        GIH = np.rad2deg(np.arcsin(asin_arg))
        Hbmcyl = np.remainder((180 - self.IGO) - GIH,360)
        a = np.sin(np.deg2rad(Hbmcyl)) - np.cos(np.deg2rad(Hbmcyl)) * np.tan(np.deg2rad(self.theta_g))

        # cylinder force (N)
        Fc = 2* (self.Ab*self.pb - self.Ar*self.pr)
        # simple lever
        simple_lever = min(0.40,np.sin(np.deg2rad(Hbmcyl)) - np.cos(np.deg2rad(Hbmcyl)) * np.tan(np.deg2rad(self.theta_g)))
        # arm mass compensate 
        link_compensate = self.k * np.cos(np.deg2rad(self.theta_g + self.beta)) / np.cos(np.deg2rad(self.theta_g))
        # load mass (kg)
        w = ((Fc * self.Lgh / self.Lag) * simple_lever) / 9.81 - link_compensate
        self.publish_output(w,Fc,link_compensate,simple_lever,a)

        # else:
            # self.get_logger().info(f"Link moving--vel: {self.omega:2f}, acc: {self.alpha:2f}", throttle_duration_sec=2)


    def publish_output(self,mass,force,arm,main,geometry):
        msg = Debug()
        msg.f = force
        msg.w = mass
        msg.theta_g = self.theta_g
        msg.theta_t = self.theta_t
        msg.pb = self.pb
        msg.pr = self.pr
        msg.arm = arm
        msg.main = main
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