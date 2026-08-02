#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np


class LoadEstimationNode(Node):
    def __init__(self):
        super().__init__('load_estimation_node')

        # measured
        self.theta_a = None             # boom angle (rad) 
        self.pb_filter = None           # bottom cylinder pressure (sensor val)
        self.pr_filter = None           # rod side pressure (sensor val)

        # constant
        self.theta_g_offset = -0.62     # boom angle offset (rad)
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

        # compensator
        self.k1 = -0.2118           # ton
        self.k2 = 1.0545e-03    
        self.k3 = 4.6246e-01        # ton/rad

        # pressure predict model
        self.duration = 1.0
        self.ts = 60.0 

        # EMA filter (cutoff freq: 0.016Hz)
        self.ema_alpha = 0.001
        self.pb_ema_filtered = None
        self.pr_ema_filtered = None

        # EMA filter for theta_a
        self.theta_a_ema_alpha = 0.1
        self.theta_a_ema_filtered = None

        self.cb_group = ReentrantCallbackGroup()

        self.create_service(
            Trigger,
            '/load_estimate',
            self.srv_callback,
            callback_group=self.cb_group
        )

        self.create_subscription(
            Float64MultiArray,
            '/vehicle/loader_joint_pressures',
            self.pressure_callback,
            10,
            callback_group=self.cb_group
        )

        self.create_subscription(
            Float64MultiArray,
            '/vehicle/loader_joint_angles',
            self.angle_callback,
            10,
            callback_group=self.cb_group
        )

        self.get_logger().info("Load estimation node initialized.")

    def angle_callback(self, msg:Float64MultiArray):
        theta_a = msg.data[0] + self.theta_g_offset

        # EMA filter for theta_a
        if self.theta_a_ema_filtered is None:
            self.theta_a_ema_filtered = theta_a
        else:
            self.theta_a_ema_filtered = self.theta_a_ema_alpha * theta_a + (1 - self.theta_a_ema_alpha) * self.theta_a_ema_filtered
        
        self.theta_a = self.theta_a_ema_filtered

    def pressure_callback(self, msg: Float64MultiArray):
        pb_raw = msg.data[0]
        pr_raw = msg.data[1]

        # EMA filter for pb
        if self.pb_ema_filtered is None:
            self.pb_ema_filtered = pb_raw
        else:
            self.pb_ema_filtered = self.ema_alpha * pb_raw + (1 - self.ema_alpha) * self.pb_ema_filtered

        # EMA filter for pr
        if self.pr_ema_filtered is None:
            self.pr_ema_filtered = pr_raw
        else:
            self.pr_ema_filtered = self.ema_alpha * pr_raw + (1 - self.ema_alpha) * self.pr_ema_filtered

        self.pb_filter = self.pb_ema_filtered
        self.pr_filter = self.pr_ema_filtered


    def pressure_estimator(self, t, pbi, pri):
        exp_term = np.exp(-t / (0.25*self.ts))
        pb_settle = (self.pb_filter - pbi * exp_term) / (1 - exp_term)
        pr_settle = (self.pr_filter - pri * exp_term) / (1 - exp_term)
        return pb_settle, pr_settle # sensor val


    def calculate_load_mass(self, pbf, prf, theta_a):
        # empty bucket offset (sensor val)
        pb_offset = 1699*(theta_a-self.theta_g_offset) + 4023
        pr_offset = 413.3

        # GEOMETRY
        Lih = np.sqrt(self.Lgh**2 + self.Lgi**2 - 2 * self.Lgh * self.Lgi * np.cos(theta_a + self.IGO))
        GIH = np.arcsin(np.clip((self.Lgh / Lih) * np.sin(theta_a + self.IGO), -1.0, 1.0))
        HIO = np.pi - self.IGO - GIH
        a = np.sin(HIO) - np.cos(HIO) * np.tan(theta_a)

        # cylinder force (N)
        pb = pbf - pb_offset
        pr = prf - pr_offset
        Fc = 2 * (self.ab * pb - self.ar * pr) * self.toPa

        simple_lever_kg = ((Fc * self.Lgh / self.Lag) * a)/9.807

        # estimated mass (ton)
        w = max(0.0, self.k1 + self.k2*simple_lever_kg + self.k3*(theta_a-self.theta_g_offset))
        return w


    def srv_callback(self, request, response):
        try:
            # Initialization
            if self.pb_filter is None or self.pr_filter is None or self.theta_a is None:
                raise Exception("Pressure or angle data not ready (filter window may be filling).")
            
            pbi, pri = self.pb_filter, self.pr_filter # Capture initial values at t=0
            pb_settle_history, pr_settle_history = [], []
            start_time = self.get_clock().now().nanoseconds / 1e9

            # Collect data and calculate settled pressure for duration
            while (self.get_clock().now().nanoseconds / 1e9 - start_time) < self.duration:
                elapsed_time = self.get_clock().now().nanoseconds / 1e9 - start_time
                if elapsed_time > 1e-9:
                    pb_settle, pr_settle = self.pressure_estimator(elapsed_time, pbi, pri)
                    pb_settle_history.append(pb_settle)
                    pr_settle_history.append(pr_settle)

            if not pb_settle_history or not pr_settle_history:
                raise Exception("Failed to collect any settled pressure data during the duration.")

            # Use median to get single value
            pbf, prf = np.median(pb_settle_history), np.median(pr_settle_history)

            # Calculate mass
            load_mass = self.calculate_load_mass(pbf, prf, self.theta_a)
            response.success = True
            response.message = f'load(ton): {load_mass:.2f}' 
        except Exception as e:
            response.success = False
            response.message = f'{str(e)}'
            self.get_logger().error(f"Load estimation failed: {str(e)}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LoadEstimationNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()