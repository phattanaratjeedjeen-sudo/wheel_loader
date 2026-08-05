#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from collections import deque
import threading
import numpy as np


class LoadEstimationNode(Node):
    def __init__(self):
        super().__init__('load_estimation_node')

        # measured
        self.theta_a = None             # boom angle (rad)
        self.theta_g_raw = None         # raw boom angle before offset (rad)
        self.pb_filter = None           # bottom cylinder pressure (sensor val)
        self.pr_filter = None           # rod side pressure (sensor val)

        # constant
        self.theta_g_offset = -0.62     # boom angle offset (rad)
        self.ab = 0.02                  # bottom cylinder area (m^2)
        self.ar = 0.014                 # rod side area (m^2)
        self.toPa = 40*10**6/32767      # convert sensor data to Pa

        # link geometry                   
        LgiX = 135/1000                               # (m) 
        LgiY = 640/1000                               # (m)
        self.Lgh = 1630/1000                          # (m)  
        self.Lag = 3929.4/1000                        # (m)
        self.Lgi = np.sqrt(LgiY**2 + LgiX**2)         # (m)  
        self.IGO = np.pi - (np.arctan2(LgiY, LgiX))   # (rad)

        # compensator
        self.k1 = -0.2353               # (ton)
        self.k2 = 1.0570e-03            # constant
        self.k3 = 4.4962e-01            # (ton/rad)

        # pressure predict model    
        self.est_time = 1.0             # duration for estimate (s)
        self.ts = 60.0                  # pressure settling time (s)

        # pressure EMA filter (cutoff freq: 0.016Hz)
        self.ema_alpha = 0.001
        self.pb_ema_filtered = None
        self.pr_ema_filtered = None

        # angle EMA filter
        self.theta_a_ema_alpha = 0.1
        self.theta_a_ema_filtered = None

        # static check
        self.vel_threshold = 0.001      # (rad/s) 
        self.acc_threshold = 0.003      # (rad/s^2)
        self.vel_a = None
        self.acc_a = None
        self.theta_a_history = deque(maxlen=3)
        self.vel_a_history = deque(maxlen=3)
        
        self.cb_group = ReentrantCallbackGroup()
        self.srv_timeout = 3.0          # (s)

        # For non-blocking service implementation
        self.srv_lock = threading.Lock()
        self.srv_wait_condition = threading.Condition(self.srv_lock)
        self.est_done_condition = threading.Condition(self.srv_lock)
        self.srv_in_progress = False
        self.theta_g_at_call = None
        self.est_timer = None
        self.pbi = None
        self.pri = None
        self.estimation_start_time = None
        self.pb_settle_history = []
        self.pr_settle_history = []


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
        self.theta_g_raw = msg.data[0]
        theta_a = self.theta_g_raw + self.theta_g_offset

        # EMA filter
        if self.theta_a_ema_filtered is None:
            self.theta_a_ema_filtered = theta_a
        else:
            self.theta_a_ema_filtered = self.theta_a_ema_alpha * theta_a + (1 - self.theta_a_ema_alpha) * self.theta_a_ema_filtered

        self.theta_a = self.theta_a_ema_filtered
        self.theta_a_history.append((current_time, self.theta_a))

        # Calculate velocity using central difference
        if len(self.theta_a_history) == 3:
            t0, y0 = self.theta_a_history[0]
            t1, _ = self.theta_a_history[1]
            t2, y2 = self.theta_a_history[2]

            dt = t2 - t0
            if dt > 1e-9:
                # Central difference for derivative at t1
                self.vel_a = (y2 - y0) / dt
                self.vel_a_history.append((t1, self.vel_a))
            else:
                self.vel_a = 0.0
        else:
            self.vel_a = None

        # Calculate acceleration using central difference
        if len(self.vel_a_history) == 3:
            t0, y0 = self.vel_a_history[0]
            t2, y2 = self.vel_a_history[2]

            dt = t2 - t0
            if dt > 1e-9:
                self.acc_a = (y2 - y0) / dt
            else:
                self.acc_a = 0.0
        else:
            self.acc_a = None

        # If a service call is waiting, check conditions and notify
        with self.srv_lock:
            if self.srv_in_progress and self.theta_g_at_call is not None:
                # Check if derivative data is available
                if self.vel_a is not None and self.acc_a is not None and self.theta_g_raw is not None:
                    is_static = abs(self.vel_a) < self.vel_threshold and abs(self.acc_a) < self.acc_threshold
                    is_downward = self.theta_g_raw < self.theta_g_at_call

                    if is_static and is_downward:
                        # Conditions are met, notify the waiting service thread
                        self.srv_wait_condition.notify()

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

    def estimation_timer_callback(self):
        with self.srv_lock:
            # Safeguard against race conditions if the timer fires after completion/cancellation
            if not self.srv_in_progress or self.estimation_start_time is None:
                return

            elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.estimation_start_time

            # Check if estimation duration has been reached
            if elapsed_time >= self.est_time:
                if self.est_timer is not None:
                    self.est_timer.cancel() # Prevent it from firing again
                self.est_done_condition.notify() # Notify the service thread that estimation is done
                return

            # Collect data point if time has passed
            if elapsed_time > 1e-9:
                if self.pb_filter is None or self.pri is None or self.pbi is None:
                    return
                pb_settle, pr_settle = self.pressure_estimator(elapsed_time, self.pbi, self.pri)
                self.pb_settle_history.append(pb_settle)
                self.pr_settle_history.append(pr_settle)


    def calculate_load_mass(self, pbf, prf, theta_a):
        # empty bucket offset (sensor val)
        pb_offset = 1699*(theta_a-self.theta_g_offset) + 4023
        pr_offset = 413.3
        pb = pbf - pb_offset
        pr = prf - pr_offset

        # geometry
        Lih = np.sqrt(self.Lgh**2 + self.Lgi**2 - 2 * self.Lgh * self.Lgi * np.cos(theta_a + self.IGO))
        GIH = np.arcsin(np.clip((self.Lgh / Lih) * np.sin(theta_a + self.IGO), -1.0, 1.0))
        HIO = np.pi - self.IGO - GIH
        a = np.sin(HIO) - np.cos(HIO) * np.tan(theta_a)

        # cylinder force (N)
        Fc = 2 * (self.ab * pb - self.ar * pr) * self.toPa  

        # estimated mass (ton)
        simple_lever_kg = ((Fc * self.Lgh / self.Lag) * a)/9.807
        w = max(0.0, self.k1 + self.k2*simple_lever_kg + self.k3*(theta_a-self.theta_g_offset))
        return w


    def srv_callback(self, request, response):
        with self.srv_lock:
            if self.srv_in_progress:
                response.success = False
                response.message = "Service is already busy. Please try again later."
                self.get_logger().warn("Received a service call while another was in progress.")
                return response
            self.srv_in_progress = True

        try:
            if self.pb_filter is None or self.pr_filter is None or self.theta_a is None or self.theta_g_raw is None:
                raise Exception("Sensor data not ready (filter window may be filling).")

            # --- Part 1: Wait for static & downward condition ---
            with self.srv_lock:
                self.theta_g_at_call = self.theta_g_raw
                self.get_logger().info(f"Service called. Waiting for static & downward condition. Initial theta_g: {self.theta_g_at_call:.3f}")
                
                wait_successful = self.srv_wait_condition.wait(timeout=self.srv_timeout)
                
                if not wait_successful:
                    raise Exception(f"Timeout: Static conditions not met within {self.srv_timeout} seconds.")

            self.get_logger().info(f"Conditions met: Static (vel={self.vel_a:.4f}, acc={self.acc_a:.4f}), Downward (theta_g={self.theta_g_raw:.3f} < {self.theta_g_at_call:.3f})")

            # --- Part 2: Estimate pressure over a duration ---
            with self.srv_lock:
                self.pbi, self.pri = self.pb_filter, self.pr_filter
                self.pb_settle_history, self.pr_settle_history = [], []
                self.estimation_start_time = self.get_clock().now().nanoseconds / 1e9
                
                # Start a timer to collect data periodically
                self.est_timer = self.create_timer(0.01, self.estimation_timer_callback, callback_group=self.cb_group)
                
                # Wait for the timer to signal completion
                estimation_successful = self.est_done_condition.wait(timeout=self.est_time + 0.5)
                
                if self.est_timer is not None:
                    self.est_timer.destroy()
                    self.est_timer = None
                
                if not estimation_successful:
                    raise Exception("Timeout: Pressure estimation period did not complete.")

            # --- Part 3: Calculate final result ---
            if not self.pb_settle_history or not self.pr_settle_history:
                raise Exception("Failed to collect any settled pressure data during the estimation duration.")

            pbf, prf = np.median(self.pb_settle_history), np.median(self.pr_settle_history)
            load_mass = self.calculate_load_mass(pbf, prf, self.theta_a)
            response.success = True
            response.message = f'load(ton): {load_mass:.2f}'

        except Exception as e:
            response.success = False
            response.message = f'{str(e)}'
            self.get_logger().error(f"Load estimation failed: {str(e)}")
        finally:
            # reset for the next service call
            with self.srv_lock:
                self.srv_in_progress = False
                self.theta_g_at_call = None
                if self.est_timer is not None:
                    self.est_timer.destroy()
                    self.est_timer = None

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