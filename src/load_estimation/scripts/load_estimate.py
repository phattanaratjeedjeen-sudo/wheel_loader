#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class LoadEstimationNode(Node):
    def __init__(self):
        super().__init__('load_estimation_node')

        # measured
        self.theta_g = None             # boom angle (rad) 
        self.pb_filter = None           # bottom cylinder pressure (sensor val)
        self.pr_filter = None           # rod side pressure (sensor val)

        self.vel_g = None               # boom angular velocity (rad/s)
        self.acc_g = None               # rad/s^2
        self.vel_threshold = 0.003      
        self.acc_threshold = 0.001      

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

        # conpensator
        self.k1 = 700.0     # slope (kg/rad)
        self.k2 = -700.0    # mass-intercept (kg)

        # pressure predict model
        self.duration = 1.0
        self.ts = 65.0 

        # median filter
        self.window_size = 100
        self.r_history = deque(maxlen=self.window_size)
        self.b_history = deque(maxlen=self.window_size)

        # EMA filter (cutoff freq: 0.016Hz)
        self.ema_alpha = 0.001
        self.pb_ema_filtered = None
        self.pr_ema_filtered = None

        # central difference derivative
        self.theta_g_history = deque(maxlen=3) # timestamp, theta_g
        self.vel_g_history = deque(maxlen=3)   # timestamp, vel_g

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

    def pressure_callback(self, msg:Float64MultiArray):
        pb_raw = msg.data[0]
        pr_raw = msg.data[1]

        # 1. Append raw values to history for median filter
        self.b_history.append(pb_raw)
        self.r_history.append(pr_raw) 
        
        if len(self.b_history) < self.window_size:
            self.pb_filter = None
            self.pr_filter = None
            return
        
        # 2. Apply median filter
        pb_median = np.median(list(self.b_history))
        pr_median = np.median(list(self.r_history))

        # 3. Apply EMA filter to the median-filtered signal
        if self.pb_ema_filtered is None:
            self.pb_ema_filtered = pb_median
            self.pr_ema_filtered = pr_median
        else:
            self.pb_ema_filtered = self.ema_alpha * pb_median + (1 - self.ema_alpha) * self.pb_ema_filtered
            self.pr_ema_filtered = self.ema_alpha * pr_median + (1 - self.ema_alpha) * self.pr_ema_filtered

        # 4. Update class attributes with final filtered pressure
        self.pb_filter = self.pb_ema_filtered
        self.pr_filter = self.pr_ema_filtered

    def pressure_estimator(self, t, pbi, pri):
        exp_term = np.exp(-t / (0.25*self.ts))
        pb_settle = (self.pb_filter - pbi * exp_term) / (1 - exp_term)
        pr_settle = (self.pr_filter - pri * exp_term) / (1 - exp_term)
        
        return pb_settle, pr_settle # sensor val


    def calculate_load_mass(self, pbf, prf, theta_g):
        # empty bucket offset (sensor val)
        pb_offset = 1699 * theta_g + 4023
        pr_offset = 413.3

        # GEOMETRY
        Lih = np.sqrt(self.Lgh**2 + self.Lgi**2 - 2 * self.Lgh * self.Lgi * np.cos(theta_g + self.IGO))
        GIH = np.arcsin(np.clip((self.Lgh / Lih) * np.sin(theta_g + self.IGO), -1.0, 1.0))
        HIO = np.pi - self.IGO - GIH
        a = np.sin(HIO) - np.cos(HIO) * np.tan(theta_g)

        # cylinder force (N)
        pb = pbf - pb_offset
        pr = prf - pr_offset
        Fc = 2 * (self.ab * pb - self.ar * pr) * self.toPa

        simple_lever = ((Fc * self.Lgh / self.Lag) * a)/9.807
        compensator = self.k1*theta_g + self.k2

        # estimated mass (kg)
        w = max(0.0,simple_lever + compensator)
        return w


    def srv_callback(self, request, response):
        try:
            # Step 1: Check for static conditions
            if self.vel_g is None or self.acc_g is None:
                raise Exception("Velocity/Acceleration not calculated yet.")

            if not (abs(self.vel_g) < self.vel_threshold and abs(self.acc_g) < self.acc_threshold):
                response.success = False
                response.message = "Static conditions not satisfied. Cannot estimate load."
                self.get_logger().warn(f"Static check failed: vel={self.vel_g:.4f}, acc={self.acc_g:.4f}")
                return response
            
            self.get_logger().info("Static conditions met. Starting load estimation process.")

            # Step 2.1: Initialization
            if self.pb_filter is None or self.pr_filter is None or self.theta_g is None:
                raise Exception("Pressure or angle data not ready (filter window may be filling).")
            
            pbi, pri, theta_gi = self.pb_filter, self.pr_filter, self.theta_g
            pb_settle_history, pr_settle_history = [], []
            start_time = self.get_clock().now().nanoseconds / 1e9

            # Step 2.2 & 2.3: Collect data and calculate settled pressure for duration
            self.get_logger().info(f"Collecting data for {self.duration} seconds...")
            while (self.get_clock().now().nanoseconds / 1e9 - start_time) < self.duration:
                elapsed_time = self.get_clock().now().nanoseconds / 1e9 - start_time
                if self.pb_filter is None or self.pr_filter is None:
                    time.sleep(0.01)
                    continue
                if elapsed_time > 1e-9:
                    pb_settle, pr_settle = self.pressure_estimator(elapsed_time, pbi, pri)
                    pb_settle_history.append(pb_settle)
                    pr_settle_history.append(pr_settle)
                time.sleep(0.01)

            if not pb_settle_history or not pr_settle_history:
                raise Exception("Failed to collect any settled pressure data during the duration.")

            # Step 2.4: Use median to get single value
            pbf, prf = np.median(pb_settle_history), np.median(pr_settle_history)
            self.get_logger().info(f"Final settled pressures (median): pbf={pbf:.2f}, prf={prf:.2f}")

            # Step 2.5: Calculate mass
            load_mass = self.calculate_load_mass(pbf, prf, theta_gi)
            response.success = True
            response.message = f'load(kg): {load_mass:.0f}' 
            self.get_logger().info(f"Estimated load: {load_mass:.0f} kg")

        except Exception as e:
            response.success = False
            response.message = f'{str(e)}'
            self.get_logger().error(f"Load estimation failed: {str(e)}")
        finally:
            # Reset for next service call
            self.get_logger().info("Resetting filters for next service call.")
            self.b_history.clear()
            self.r_history.clear()
            self.pb_ema_filtered = None
            self.pr_ema_filtered = None
            self.pb_filter = None
            self.pr_filter = None
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