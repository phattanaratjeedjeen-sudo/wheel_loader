#!/usr/bin/python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.widgets import Slider, CheckButtons, Button, TextBox
from tkinter import filedialog
import re
from scipy.optimize import curve_fit
from scipy import stats

# --- Constants ---
Lgh = 0.163
Lgi = np.sqrt(0.135**2 + 0.64**2)
Lag = 3.9294
IGO = np.pi - np.arctan2(0.64, 0.135)
toPa = 40 * 10**6 / (2**15 / 2 - 1)

# Cylinder areas
Ab = 0.02  # m^2, bottom cross-section area

# --- Thresholds for Static Condition ---
ACC_THRESHOLD = 0.001  # rad/s^2
VEL_THRESHOLD = 0.003  # rad/s

# Column names from the CSV file
col_theta_g = '/debug/theta_g'
col_acc_g = 'acc_g'
col_vel_g = 'vel_g'
col_pb = '/debug/pb'
col_pr = '/debug/pr'
col_w_calculated = 'w_calculated'


class InteractiveTuner:
    """
    A class to create an interactive plot for tuning compensation parameters.
    """
    def __init__(self, df, current_filename):
        self.df = df.copy()
        self.current_filename = current_filename
        self.use_compensation = True
        self.k1 = 1640.0
        self.k2 = 0.225
        self.k3 = 0.0
        self.k4 = 0.0
        self.use_bucket_compensation = False
        self.Ar = 0.014  # m^2, piston cross-section area
        self.use_median_filter = True
        self.use_ema_filter = True
        self.ema_alpha = 0.001
        self.median_window = 100
        self.slope_time_interval = 0.1
        self.initial_search_time = 0.0
        self.slope_threshold = 1.0
        self.settling_time = None
        self.settling_duration = None
        self.use_auto_initial_time = True
        self.use_slope_filter = True
        self.slope_ema_alpha = 0.001
        self.ts_param = 80.92
        self.duration_param = 3.0

        # --- Parse filename for target_theta_g ---
        match = re.search(r'(\d+\.\d+)', self.current_filename)
        default_target = 0.3
        if match:
            try:
                self.target_theta_g = float(match.group(1))
                print(f"Auto-set target_theta_g from filename to: {self.target_theta_g}")
            except (ValueError, IndexError):
                self.target_theta_g = default_target
                print(f"Could not parse float from filename. Defaulting target_theta_g to: {default_target}")
        else:
            self.target_theta_g = default_target
            print(f"Filename does not contain a float. Defaulting target_theta_g to: {default_target}")

        self.theta_g_threshold = 0.05
        self.vel_g_threshold_auto = VEL_THRESHOLD
        self.acc_g_threshold_auto = ACC_THRESHOLD

        # --- Create Figure and Axes for plots---
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
        self.fig.subplots_adjust(left=0.05, bottom=0.05, right=0.99, top=0.94, wspace=0.08, hspace=0.2)

        # --- Create a new Figure for the time plot ---
        self.time_fig, self.ax_time = plt.subplots(1, 1, num='Figure 2: Load vs Time', figsize=(12, 6))
        self.time_fig.subplots_adjust(left=0.05, bottom=0.05, right=0.99, top=0.94, wspace=0.08, hspace=0.2)

        # --- Create a new Figure for pressure analysis ---
        self.pressure_fig = plt.figure(num='Figure 3: Pressure Analysis', figsize=(12, 8))
        gs = self.pressure_fig.add_gridspec(2, 2)
        self.ax_pressure_pb = self.pressure_fig.add_subplot(gs[0, 0])
        self.ax_theta_g_t = self.pressure_fig.add_subplot(gs[0, 1], sharex=self.ax_pressure_pb)
        self.ax_slope_pb = self.pressure_fig.add_subplot(gs[1, :], sharex=self.ax_pressure_pb)
        self.pressure_fig.subplots_adjust(left=0.05, bottom=0.05, right=0.99, top=0.94, wspace=0.08, hspace=0.2)

        # --- Create a new Figure for final pressure analysis ---
        self.final_pressure_fig, self.ax_final_pressure = plt.subplots(num='Figure 4: Final Pressure', figsize=(12, 8))
        self.final_pressure_fig.subplots_adjust(left=0.05, bottom=0.05, right=0.99, top=0.94, wspace=0.08, hspace=0.2)


        # --- Create a separate Figure for controls ---
        self.control_fig = plt.figure("Tuning Panel", figsize=(7, 10.0))
        self.control_fig.subplots_adjust(left=0.05, right=0.95, top=0.98, bottom=0.02)

        # --- Create UI Controls in the new window, arranged top-to-bottom ---
        # This procedural layout makes adding/removing widgets easier.
        # Each "Row" is a block. Modifying one block won't break others.
        y_cursor = 1.0 # Start from top

        # --- Layout Parameters ---
        left_margin = 0.05
        right_margin = 0.05
        content_width = 1.0 - left_margin - right_margin
        
        # Heights
        h_widget = 0.028
        h_checkbox = 0.028
        h_checkbox_double = h_checkbox * 2
        h_checkbox_triple = h_checkbox * 3 + 0.01

        # Gaps
        v_gap_widget = 0.008
        v_gap_section = 0.025
        h_gap_widget = 0.03

        # Proportions for common layouts
        prop_checkbox = 0.25
        prop_slider = 1.0 - prop_checkbox - h_gap_widget
        prop_half = 0.5 - h_gap_widget / 2

        # --- Layout Helper Function ---
        def add_section_title(title):
            nonlocal y_cursor
            y_cursor -= v_gap_section
            self.control_fig.text(left_margin, y_cursor, title, fontsize=10, weight='bold')
            y_cursor -= (h_widget + v_gap_widget) # Add space after title

        # --- Section 1: File & View ---
        add_section_title('File & View Controls')
        row_h = h_checkbox_triple
        y_cursor -= row_h
        ax_select_btn = self.control_fig.add_axes([left_margin, y_cursor + row_h - h_widget, content_width * 0.4, h_widget])
        ax_fig_vis = self.control_fig.add_axes([left_margin + content_width * 0.4 + h_gap_widget, y_cursor, content_width * 0.5, row_h])
        y_cursor -= v_gap_widget

        # --- Section 2: Signal Filtering ---
        add_section_title('Pressure Signal Filtering')
        row_h = h_checkbox_double
        y_cursor -= row_h
        slider_x = left_margin + content_width * prop_checkbox + h_gap_widget
        slider_w = content_width * prop_slider
        ax_filter_checks = self.control_fig.add_axes([left_margin, y_cursor, content_width * prop_checkbox, row_h])
        ax_median_win = self.control_fig.add_axes([slider_x, y_cursor + h_widget, slider_w, h_widget])
        ax_ema_alpha = self.control_fig.add_axes([slider_x, y_cursor, slider_w, h_widget])
        y_cursor -= v_gap_widget

        # --- Section 3: Load Model Compensation ---
        add_section_title('Load Model Compensation')
        y_cursor -= h_widget; ax_check_link = self.control_fig.add_axes([left_margin, y_cursor, content_width * prop_checkbox, h_widget]); ax_k1 = self.control_fig.add_axes([slider_x, y_cursor, slider_w, h_widget]); y_cursor -= v_gap_widget
        y_cursor -= h_widget; ax_k2 = self.control_fig.add_axes([slider_x, y_cursor, slider_w, h_widget]); y_cursor -= v_gap_widget
        y_cursor -= h_widget; ax_check_bucket = self.control_fig.add_axes([left_margin, y_cursor, content_width * prop_checkbox, h_widget]); ax_k3 = self.control_fig.add_axes([slider_x, y_cursor, slider_w, h_widget]); y_cursor -= v_gap_widget
        y_cursor -= h_widget; ax_k4 = self.control_fig.add_axes([slider_x, y_cursor, slider_w, h_widget]); y_cursor -= v_gap_widget

        # --- Section 4: Settling Time Analysis ---
        add_section_title('Settling Time Analysis')
        y_cursor -= h_widget; ax_check_auto_time = self.control_fig.add_axes([left_margin, y_cursor, content_width, h_widget]); y_cursor -= v_gap_widget
        
        y_cursor -= h_widget
        textbox_x = left_margin + content_width * 0.1
        textbox_w = content_width * (prop_half - 0.1)
        ax_target_theta = self.control_fig.add_axes([textbox_x, y_cursor, textbox_w, h_widget])
        ax_theta_thresh = self.control_fig.add_axes([textbox_x + textbox_w + h_gap_widget, y_cursor, textbox_w, h_widget])
        y_cursor -= v_gap_widget

        y_cursor -= h_widget
        ax_vel_thresh_auto = self.control_fig.add_axes([textbox_x, y_cursor, textbox_w, h_widget])
        ax_acc_thresh_auto = self.control_fig.add_axes([textbox_x + textbox_w + h_gap_widget, y_cursor, textbox_w, h_widget])
        y_cursor -= v_gap_widget

        y_cursor -= h_widget
        ax_check_slope_filter = self.control_fig.add_axes([left_margin, y_cursor, content_width * prop_checkbox, h_widget])
        ax_slope_ema_alpha = self.control_fig.add_axes([slider_x, y_cursor, slider_w, h_widget])
        y_cursor -= v_gap_widget

        y_cursor -= h_widget; ax_initial_time = self.control_fig.add_axes([slider_x, y_cursor, slider_w, h_widget]); y_cursor -= v_gap_widget
        y_cursor -= h_widget; ax_slope_interval = self.control_fig.add_axes([slider_x, y_cursor, slider_w, h_widget]); y_cursor -= v_gap_widget
        y_cursor -= h_widget; ax_slope_thresh = self.control_fig.add_axes([slider_x, y_cursor, slider_w, h_widget]); y_cursor -= v_gap_widget

        # --- Section 5: Final Pressure Analysis ---
        add_section_title('Final Pressure Analysis')
        y_cursor -= h_widget
        textbox_x_final = left_margin + content_width * 0.1
        textbox_w_final = content_width * (prop_half - 0.1)
        ax_ts_param = self.control_fig.add_axes([textbox_x_final, y_cursor, textbox_w_final, h_widget])
        ax_duration_param = self.control_fig.add_axes([textbox_x_final + textbox_w_final + h_gap_widget, y_cursor, textbox_w_final, h_widget])
        y_cursor -= v_gap_widget

        # --- Instantiate Widgets (in section order) ---
        # Section 1: File & View
        self.btn_select_file = Button(ax_select_btn, 'Select CSV File')
        self.check_figs = CheckButtons(ax_fig_vis, ['Load(θg)', 'Load(t)', 'Pressure(t)', 'Final Pressure'], actives=[False, False, False, False])

        # Section 2: Signal Filtering
        self.check_filters = CheckButtons(ax_filter_checks, ['Median', 'EMA'], actives=[self.use_median_filter, self.use_ema_filter])
        self.slider_median_win = Slider(ax_median_win, 'Median Win', 51, 201, valinit=self.median_window, valstep=10)
        self.slider_ema_alpha = Slider(ax_ema_alpha, 'EMA Alpha', 0.001, 0.01, valinit=self.ema_alpha, valstep=0.001)

        # Section 3: Load Model Compensation
        self.check_comp = CheckButtons(ax_check_link, ['Link Comp.'], [self.use_compensation])
        self.slider_k1 = Slider(ax_k1, 'k1', 0, 5000, valinit=self.k1)
        self.slider_k2 = Slider(ax_k2, 'k2', 0.0, 1.0, valinit=self.k2)
        self.check_bucket_comp = CheckButtons(ax_check_bucket, ['Bucket Comp.'], [self.use_bucket_compensation])
        self.slider_k3 = Slider(ax_k3, 'k3', 0, 5000, valinit=self.k3)
        self.slider_k4 = Slider(ax_k4, 'k4', -1.0, 1.0, valinit=self.k4)

        # Section 4: Settling Time Analysis
        self.check_auto_time = CheckButtons(ax_check_auto_time, ['Auto-detect Initial Time'], [self.use_auto_initial_time])
        self.text_target_theta = TextBox(ax_target_theta, 'Target θg', initial=str(self.target_theta_g))
        self.text_theta_thresh = TextBox(ax_theta_thresh, 'θg Thresh', initial=str(self.theta_g_threshold))
        self.text_vel_thresh = TextBox(ax_vel_thresh_auto, 'Vel Thresh', initial=str(self.vel_g_threshold_auto))
        self.text_acc_thresh = TextBox(ax_acc_thresh_auto, 'Acc Thresh', initial=str(self.acc_g_threshold_auto))
        self.check_slope_filter = CheckButtons(ax_check_slope_filter, ['Filter Slope (EMA)'], [self.use_slope_filter])
        self.text_slope_ema_alpha = TextBox(ax_slope_ema_alpha, 'Slope EMA α', initial=str(self.slope_ema_alpha))
        self.slider_initial_time = Slider(ax_initial_time, 'Initial Time (s)', 0.0, 100.0, valinit=self.initial_search_time, valstep=1.0)
        self.slider_slope_interval = Slider(ax_slope_interval, 'Slope Interval (s)', 0.1, 5.0, valinit=self.slope_time_interval, valstep=0.1)
        self.slider_slope_thresh = Slider(ax_slope_thresh, 'Slope Thresh', 0.0, 50.0, valinit=self.slope_threshold, valstep=1.0)

        # Section 5: Final Pressure Analysis
        self.text_ts_param = TextBox(ax_ts_param, 'Ts (sec)', initial=str(self.ts_param))
        self.text_duration_param = TextBox(ax_duration_param, 'Duration (sec)', initial=str(self.duration_param))

        # --- Connect UI Events ---
        self.btn_select_file.on_clicked(self.open_file_dialog)
        self.slider_k1.on_changed(self.update_params)
        self.slider_k2.on_changed(self.update_params)
        self.slider_k3.on_changed(self.update_params)
        self.slider_k4.on_changed(self.update_params)
        self.check_comp.on_clicked(self.toggle_compensation)
        self.check_bucket_comp.on_clicked(self.toggle_bucket_compensation)
        self.check_figs.on_clicked(self.toggle_figure_visibility)
        self.check_filters.on_clicked(self.toggle_filters)
        self.slider_ema_alpha.on_changed(self.update_filter_params)
        self.slider_median_win.on_changed(self.update_filter_params)
        self.slider_slope_interval.on_changed(self.update_slope_params)
        self.slider_initial_time.on_changed(self.update_settling_params)
        self.slider_slope_thresh.on_changed(self.update_settling_params)
        self.check_auto_time.on_clicked(self.toggle_auto_initial_time)
        self.text_target_theta.on_submit(self.update_auto_time_params)
        self.text_theta_thresh.on_submit(self.update_auto_time_params)
        self.text_vel_thresh.on_submit(self.update_auto_time_params)
        self.text_acc_thresh.on_submit(self.update_auto_time_params)
        self.check_slope_filter.on_clicked(self.toggle_slope_filter)
        self.text_slope_ema_alpha.on_submit(self.update_slope_filter_params)
        self.text_ts_param.on_submit(self.update_final_pressure_params)
        self.text_duration_param.on_submit(self.update_final_pressure_params)

        # --- Initial Calculation and Plot ---
        self.recalculate_and_plot()

        # Hide all plot figures by default on startup
        self.toggle_figure_visibility(None)

    def calculate_mass(self, theta_g, Fc):
        Lih = np.sqrt(Lgh**2 + Lgi**2 - 2 * Lgh * Lgi * np.cos(theta_g + IGO))
        GIH = np.arcsin(np.clip((Lgh / Lih) * np.sin(theta_g + IGO), -1.0, 1.0))
        Hbmcyl = np.pi - IGO - GIH
        a = np.sin(Hbmcyl) - np.cos(Hbmcyl) * np.tan(theta_g)
        simple_lever = ((Fc * Lgh / Lag) * a) / 9.807

        w = simple_lever
        if self.use_compensation:
            link_compensate = self.k1 * np.cos(theta_g + self.k2) / np.cos(theta_g)
            w -= link_compensate

        if self.use_bucket_compensation:
            # Replicating logic from test5.py, using np.minimum for vectorization
            # This term can be sensitive, handle potential warnings
            with np.errstate(invalid='ignore'):  # tan can be inf
                bucket_compensate = self.k3 * np.cos(theta_g + self.k4) / np.cos(theta_g)
            w -= bucket_compensate

        return w

    def recalculate_and_plot(self):
        # --- Perform Calculation ---
        theta_g = self.df[col_theta_g]

        # --- Apply Filters ---
        pb_filtered = self.df[col_pb].copy()
        pr_filtered = self.df[col_pr].copy()
        
        applied_filters = []

        if self.use_median_filter:
            win = int(self.median_window)
            # Ensure window is odd
            if win % 2 == 0:
                win += 1
            pb_filtered = pb_filtered.rolling(window=win, min_periods=1, center=True).median().bfill().ffill()
            pr_filtered = pr_filtered.rolling(window=win, min_periods=1, center=True).median().bfill().ffill()
            applied_filters.append('Median')

        if self.use_ema_filter:
            pb_filtered = pb_filtered.ewm(alpha=self.ema_alpha, adjust=False).mean()
            pr_filtered = pr_filtered.ewm(alpha=self.ema_alpha, adjust=False).mean()
            applied_filters.append('EMA')
        
        self.applied_filters_str = ' -> '.join(applied_filters) if applied_filters else 'None'
        # --- Calculate Force ---
        # Calculate force from (filtered) pressure columns
        Fc = 2 * (Ab * pb_filtered - self.Ar * pr_filtered) * toPa
        self.df[col_w_calculated] = self.calculate_mass(theta_g, Fc)

        # --- Calculate Slopes for plotting ---
        # Calculate the slope over a user-defined interval.
        time_interval_seconds = self.slope_time_interval
        
        # Determine the number of samples that correspond to the desired time interval
        avg_dt = self.df['time'].diff().mean()
        if pd.isna(avg_dt) or avg_dt < 1e-6:
            samples_per_interval = 10 # Fallback to 10Hz assumption if dt is invalid
        else:
            samples_per_interval = max(1, int(round(time_interval_seconds / avg_dt)))

        # Calculate slope: (change in pressure) / (change in time) over the interval.
        time_delta = self.df['time'].diff(periods=samples_per_interval).replace(0, 1e-9) # Avoid division by zero
        slope_pb = pb_filtered.diff(periods=samples_per_interval) / time_delta

        # --- Filter the slope if enabled ---
        slope_to_analyze = slope_pb.copy()
        if self.use_slope_filter:
            slope_to_analyze = slope_to_analyze.ewm(alpha=self.slope_ema_alpha, adjust=False).mean()

        # --- Find Settling Time ---
        self.settling_time = None
        self.settling_duration = None

        # Create a mask for the time range to search
        search_mask = self.df['time'] >= self.initial_search_time


        # Check if there's any data in the search range
        if search_mask.any():
            # Find integer locations of points within the search range where the slope exceeds the threshold
            unstable_ilocs = np.where(search_mask & (slope_to_analyze.abs() > self.slope_threshold))[0]

            if len(unstable_ilocs) == 0:
                # If no points exceed the threshold, it's settled from the start of the search range.
                # Find the first valid index in the search mask.
                first_search_iloc = np.where(search_mask)[0][0]
                self.settling_time = self.df.iloc[first_search_iloc]['time']
            else:
                # The settling point is the one immediately after the last unstable point
                last_unstable_iloc = unstable_ilocs[-1]
                settling_iloc = last_unstable_iloc + 1

                if settling_iloc < len(self.df):
                    self.settling_time = self.df.iloc[settling_iloc]['time']
                # else: no settling point found within the dataframe

            if self.settling_time is not None:
                # Ensure settling_duration is a scalar float
                duration = self.settling_time - self.initial_search_time
                self.settling_duration = max(0.0, float(duration))

        static_df = self.df[
            (self.df[col_vel_g].abs() < VEL_THRESHOLD) &
            (self.df[col_acc_g].abs() < ACC_THRESHOLD)
        ].copy()

        # --- Clear and Redraw Plots ---
        self.ax1.clear()
        self.ax2.clear()
        self.ax_time.clear()
        self.ax_pressure_pb.clear()
        self.ax_theta_g_t.clear()
        self.ax_slope_pb.clear()
        self.ax_final_pressure.clear()

        # Update the main plot window's title
        self.fig.suptitle(f'Load Estimation Analysis for "{self.current_filename}"', fontsize=16)
        # --- Subplot 1: All Data ---
        self.ax1.scatter(self.df[col_theta_g], self.df[col_w_calculated], alpha=0.6, s=10)
        self.ax1.set_title('All Data Points')
        self.ax1.set_ylabel('Estimated Mass (w) [kg]')
        self.ax1.grid(True)

        # --- Subplot 2: Static Data Only ---
        if not static_df.empty:
            # --- Stats for w calculated by this script ---
            w_calc_series = static_df[col_w_calculated]
            avg_w_calc = w_calc_series.mean()
            std_w_calc = w_calc_series.std()

            stats_text = (f'Live Calc -> Avg: {avg_w_calc:.1f}, Std: {std_w_calc:.1f}')

            # Plot w calculated by this script
            self.ax2.scatter(static_df[col_theta_g], w_calc_series, alpha=0.5, s=15,
                             color='green', marker='o', label='w (Live Calc)')

            # Add lines for actual load and averages
            # The 'Actual Load' line has been removed as it was tied to the hardcoded file list.
            self.ax2.axhline(y=avg_w_calc, color='green', linestyle='--', linewidth=2, label=f'Live Calc Avg: {avg_w_calc:.1f} kg')

            # Add a shaded region for the standard deviation of the live calculation
            self.ax2.axhspan(avg_w_calc - std_w_calc, avg_w_calc + std_w_calc, color='green', alpha=0.15, label=f'Live Calc Std Dev')
            self.ax2.set_title(f'Static Data: Live Calculation\n{stats_text}')
            self.ax2.legend(fontsize='small')
        else:
            self.ax2.set_title('No Static Data Found for Estimated Mass')
            self.ax2.text(0.5, 0.5, 'No data points met the static criteria.',
                     horizontalalignment='center', verticalalignment='center',
                     transform=self.ax2.transAxes)


        self.ax2.set_ylabel('Estimated Mass (w) [kg]')
        self.ax2.grid(True)

        # --- Subplot 3 is now commented out ---

        # --- Plot for Load vs Time in a separate window ---
        self.time_fig.suptitle(f'Figure 2: Estimated Load vs. Time for "{self.current_filename}"')
        if 'time' in self.df.columns:
            # --- Subplot 1: Load vs Time ---
            self.ax_time.plot(self.df['time'], self.df[col_w_calculated], label='Live Calc Mass (w)')
            # The 'Actual Load' line has been removed as it was tied to the hardcoded file list.

            self.ax_time.legend()
            self.ax_time.grid(True)
            self.ax_time.set_title('Estimated Load vs. Time')
            self.ax_time.set_ylabel('Estimated Mass (w) [kg]')
            self.ax_time.set_xlabel('Time (s)')

        else:
            self.ax_time.text(0.5, 0.5, "'__time' column not found in CSV.",
                              horizontalalignment='center', verticalalignment='center',
                              transform=self.ax_time.transAxes)

        # --- Plot for Pressure Analysis in a separate window ---
        self.pressure_fig.suptitle(f'Figure 3: Pressure Analysis for "{self.current_filename}"')
        if 'time' in self.df.columns:
            # --- Top-left: Raw vs Filtered Pressure (pb) ---
            self.ax_pressure_pb.plot(self.df['time'], self.df[col_pb], label='Raw pb', color='cyan', alpha=0.7)
            self.ax_pressure_pb.plot(self.df['time'], pb_filtered, label=f'Filtered pb ({self.applied_filters_str})', color='blue')
            self.ax_pressure_pb.axvline(x=self.initial_search_time, color='orange', linestyle=':', lw=2, label=f'Search Start: {self.initial_search_time:.1f}s')

            # Add text for pressure at start time
            start_idx = self.df['time'].sub(self.initial_search_time).abs().idxmin()
            pb_at_start = pb_filtered.loc[start_idx]
            self.ax_pressure_pb.text(self.initial_search_time, pb_at_start, f' {pb_at_start:.1f}', color='orange', ha='left', va='bottom', bbox=dict(facecolor='white', alpha=0.5, edgecolor='none', pad=0.1))

            if self.settling_time is not None:
                self.ax_pressure_pb.axvline(x=self.settling_time, color='green', linestyle='--', lw=2, label=f'Settling Time: {self.settling_time:.2f}s')

                # Add text for pressure at settling time
                settle_idx = self.df['time'].sub(self.settling_time).abs().idxmin()
                pb_at_settle = pb_filtered.loc[settle_idx]
                self.ax_pressure_pb.text(self.settling_time, pb_at_settle, f' {pb_at_settle:.1f}', color='green', ha='left', va='top', bbox=dict(facecolor='white', alpha=0.5, edgecolor='none', pad=0.1))

            self.ax_pressure_pb.set_title('Raw vs. Filtered Pressure (pb)')
            self.ax_pressure_pb.set_ylabel('Pressure (ADC value)')
            self.ax_pressure_pb.legend(fontsize='small')
            self.ax_pressure_pb.grid(True)

            # --- Top-right: Boom Angle (theta_g) vs Time ---
            self.ax_theta_g_t.plot(self.df['time'], self.df[col_theta_g], label='θg', color='purple')
            self.ax_theta_g_t.axvline(x=self.initial_search_time, color='orange', linestyle=':', lw=2, label=f'Search Start: {self.initial_search_time:.1f}s')
            if self.settling_time is not None:
                self.ax_theta_g_t.axvline(x=self.settling_time, color='green', linestyle='--', lw=2, label=f'Settling Time: {self.settling_time:.2f}s')
            self.ax_theta_g_t.set_title('Boom Angle (θg) vs. Time')
            self.ax_theta_g_t.set_ylabel('Angle (rad)')
            self.ax_theta_g_t.legend(fontsize='small')
            self.ax_theta_g_t.grid(True)

            # --- Bottom row: Slope of pb ---
            title_slope_pb = 'Slope of Filtered pb (for Settling Time)'
            if self.use_slope_filter:
                self.ax_slope_pb.plot(self.df['time'], slope_pb, label='Unfiltered Slope', color='blue', alpha=0.4)
                self.ax_slope_pb.plot(self.df['time'], slope_to_analyze, label='Filtered Slope (EMA)', color='red')
            else:
                self.ax_slope_pb.plot(self.df['time'], slope_pb, label='Slope pb', color='blue')

            self.ax_slope_pb.axhline(y=self.slope_threshold, color='gray', linestyle='--', lw=1)
            self.ax_slope_pb.axhline(y=-self.slope_threshold, color='gray', linestyle='--', lw=1, label=f'Threshold ({self.slope_threshold:.1f})')
            self.ax_slope_pb.axvline(x=self.initial_search_time, color='orange', linestyle=':', lw=2, label=f'Search Start: {self.initial_search_time:.1f}s')

            if self.settling_time is not None:
                self.ax_slope_pb.axvline(x=self.settling_time, color='green', linestyle='--', lw=2, label=f'Settling Time: {self.settling_time:.2f}s')
                title_slope_pb += f'\nTime to Settle: {self.settling_duration:.2f}s'
            elif search_mask.any():
                title_slope_pb += '\n(No settling point found)'

            self.ax_slope_pb.set_title(title_slope_pb)
            self.ax_slope_pb.set_ylabel('Pressure Slope')
            self.ax_slope_pb.set_xlabel('Time (s)')
            self.ax_slope_pb.legend(fontsize='small')
            self.ax_slope_pb.grid(True)

        # --- Plot for Final Pressure Analysis in a separate window ---
        self.final_pressure_fig.suptitle(f'Figure 4: Final Pressure Analysis for "{self.current_filename}"')
        
        # Define the analysis window
        analysis_start_time = self.initial_search_time
        analysis_end_time = analysis_start_time + self.duration_param
        
        analysis_mask = (self.df['time'] >= analysis_start_time) & (self.df['time'] <= analysis_end_time)
        analysis_df = self.df[analysis_mask].copy()
        
        if not analysis_df.empty and len(analysis_df) > 1:
            # 1. Set time at search start time to 0
            analysis_df['t_shifted'] = analysis_df['time'] - analysis_start_time
            pb_window = pb_filtered[analysis_mask]
            
            # Get initial pressure
            pbi = pb_window.iloc[0]

            # For calculations, use numpy arrays to avoid potential type issues with linters
            t_np = analysis_df['t_shifted'].to_numpy(dtype=float)
            pb_window_np = pb_window.to_numpy(dtype=float)
            
            # 4. Calculate pbf
            tau = 0.25 * self.ts_param
            
            with np.errstate(divide='ignore', invalid='ignore'):
                exp_term = np.exp(-t_np / tau)
                pbf_float = (pb_window_np - pbi * exp_term) / (1 - exp_term)
            
            # 5. Convert pbf to integer
            pbf_int = np.round(pbf_float[1:])
            t_pbf = t_np[1:]
            
            # 6. Calculate statistics
            if len(pbf_int) > 0:
                pbf_mean = np.mean(pbf_int)
                pbf_median = np.median(pbf_int)
                pbf_std = np.std(pbf_int)
                pbf_mode_result = stats.mode(pbf_int, keepdims=True)
                pbf_mode = str(pbf_mode_result.mode[0])
                
                stats_text = (f"Stats for pbf:\n"
                              f"Mean: {pbf_mean:.2f}\n"
                              f"Median: {pbf_median:.2f}\n"
                              f"Mode: {pbf_mode}\n"
                              f"Std Dev: {pbf_std:.2f}")
            else:
                stats_text = "Not enough data for pbf stats."

            # 7. Generate chart pbf(t)
            self.ax_final_pressure.plot(t_pbf, pbf_int, marker='o', linestyle='-', label='pbf (integer)')
            
            # 8. Exponential curve fitting
            def exp_func(x, a, b, c):
                return a * np.exp(b * x) + c

            try:
                # A more robust initial guess for the curve fit:
                # a: initial amplitude (initial value - final value)
                # b: decay rate (a small negative number)
                # c: final offset (the value it settles to)
                p0 = (pb_window_np[0] - pb_window_np[-1], -0.1, pb_window_np[-1])
                popt, _ = curve_fit(exp_func, t_np, pb_window_np, p0=p0, maxfev=5000, bounds=([-np.inf, -np.inf, 0], [np.inf, 0, np.inf]))

                # Plot the final pressure from the curve fit as a horizontal line for comparison
                final_pressure_fit = popt[2]
                self.ax_final_pressure.axhline(y=final_pressure_fit, color='red', linestyle='--', label=f'Fit Final Pressure: {final_pressure_fit:.2f}')

                residuals = pb_window_np - exp_func(t_np, *popt)
                ss_res = np.sum(residuals**2)
                ss_tot = np.sum((pb_window_np - np.mean(pb_window_np))**2)
                r_squared = 1 - (ss_res / ss_tot)
                eq_text = (f"Fit: y = {popt[0]:.2f} * e^({popt[1]:.2f}*t) + {final_pressure_fit:.2f}\n"
                           f"R² = {r_squared:.4f}")
                print(f"--- Figure 4 Curve Fit ---\n{eq_text}\n")
            except (RuntimeError, ValueError):
                eq_text = "Curve fit failed."
                print("--- Figure 4 Curve Fit ---\nCurve fit failed.\n")
            
            self.ax_final_pressure.text(0.05, 0.95, stats_text, transform=self.ax_final_pressure.transAxes, fontsize=10, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            self.ax_final_pressure.text(0.95, 0.95, eq_text, transform=self.ax_final_pressure.transAxes, fontsize=10, verticalalignment='top', horizontalalignment='right', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
            self.ax_final_pressure.set_title('pbf Analysis')
            self.ax_final_pressure.set_xlabel('Time since Search Start (s)')
            self.ax_final_pressure.set_ylabel('pbf (ADC value, integer)')
            self.ax_final_pressure.legend()
            self.ax_final_pressure.grid(True)
        else:
            self.ax_final_pressure.text(0.5, 0.5, "No data in analysis window.", horizontalalignment='center', verticalalignment='center', transform=self.ax_final_pressure.transAxes)

        self.time_fig.canvas.draw_idle()
        self.pressure_fig.canvas.draw_idle()
        self.fig.canvas.draw_idle()
        self.final_pressure_fig.canvas.draw_idle()

    def update_params(self, val):
        self.k1 = self.slider_k1.val
        self.k2 = self.slider_k2.val
        self.k3 = self.slider_k3.val
        self.k4 = self.slider_k4.val
        self.recalculate_and_plot()

    def toggle_compensation(self, label):
        self.use_compensation = self.check_comp.get_status()[0]
        self.recalculate_and_plot()

    def toggle_bucket_compensation(self, label):
        self.use_bucket_compensation = self.check_bucket_comp.get_status()[0]
        self.recalculate_and_plot()

    def toggle_figure_visibility(self, label):
        """
        Shows or hides figures based on the check buttons.
        NOTE: This uses a backend-specific method (for TkAgg) to show/hide windows
        without closing them, which is cleaner than closing and recreating figures.
        It may not work with other matplotlib backends.
        """
        main_vis, time_vis, pressure_vis, final_pressure_vis = self.check_figs.get_status()

        def _toggle_win(fig, is_visible):
            """Helper to safely toggle a window's visibility."""
            manager = getattr(fig.canvas, 'manager', None)
            if manager and hasattr(manager, 'window') and manager.window:
                if is_visible:
                    manager.window.deiconify()
                else:
                    manager.window.withdraw()

        _toggle_win(self.fig, main_vis)
        _toggle_win(self.time_fig, time_vis)
        _toggle_win(self.pressure_fig, pressure_vis)
        _toggle_win(self.final_pressure_fig, final_pressure_vis)

    def toggle_filters(self, label):
        self.use_median_filter, self.use_ema_filter = self.check_filters.get_status()
        self.recalculate_and_plot()

    def update_filter_params(self, val):
        self.ema_alpha = self.slider_ema_alpha.val
        self.median_window = self.slider_median_win.val
        self.recalculate_and_plot()

    def update_slope_filter_params(self, text):
        try:
            self.slope_ema_alpha = float(self.text_slope_ema_alpha.text)
        except ValueError:
            print(f"Invalid input for Slope EMA Alpha: '{text}'. Please enter a valid number.")
            return
        self.recalculate_and_plot()

    def update_final_pressure_params(self, text):
        try:
            self.ts_param = float(self.text_ts_param.text)
            self.duration_param = float(self.text_duration_param.text)
        except ValueError:
            print(f"Invalid input for Final Pressure params: '{text}'. Please enter valid numbers.")
            return
        self.recalculate_and_plot()

    def update_slope_params(self, val):
        self.slope_time_interval = self.slider_slope_interval.val
        self.recalculate_and_plot()

    def update_settling_params(self, val):
        self.initial_search_time = self.slider_initial_time.val
        self.slope_threshold = self.slider_slope_thresh.val
        self.recalculate_and_plot()

    def toggle_auto_initial_time(self, label):
        self.use_auto_initial_time = self.check_auto_time.get_status()[0]
        is_manual_mode = not self.use_auto_initial_time
        self.slider_initial_time.set_active(is_manual_mode)

        if self.use_auto_initial_time:
            self.run_auto_initial_time_detection()
        else:
            self.recalculate_and_plot()

    def update_auto_time_params(self, text):
        try:
            self.target_theta_g = float(self.text_target_theta.text)
            self.theta_g_threshold = float(self.text_theta_thresh.text)
            self.vel_g_threshold_auto = float(self.text_vel_thresh.text)
            self.acc_g_threshold_auto = float(self.text_acc_thresh.text)
        except ValueError:
            print(f"Invalid input: '{text}'. Please enter a valid number.")
            return
        if self.use_auto_initial_time:
            self.run_auto_initial_time_detection()

    def toggle_slope_filter(self, label):
        self.use_slope_filter = self.check_slope_filter.get_status()[0]
        self.recalculate_and_plot()

    def run_auto_initial_time_detection(self):
        cond_theta = (self.df[col_theta_g] >= self.target_theta_g - self.theta_g_threshold) & \
                     (self.df[col_theta_g] <= self.target_theta_g + self.theta_g_threshold)
        cond_vel = self.df[col_vel_g].abs() < self.vel_g_threshold_auto
        cond_acc = self.df[col_acc_g].abs() < self.acc_g_threshold_auto

        first_match_df = self.df[cond_theta & cond_vel & cond_acc]

        if not first_match_df.empty:
            auto_initial_time = first_match_df['time'].iloc[0]
            self.slider_initial_time.set_val(auto_initial_time)
        else:
            print("Auto-detect: No time found that satisfies all conditions.")

    def load_file(self, csv_file_path):
        """Loads a CSV file, processes it, and triggers a replot."""
        try:
            df = pd.read_csv(csv_file_path)
            self.current_filename = os.path.basename(csv_file_path).replace('.csv', '')
            print(f"\nSuccessfully loaded '{csv_file_path}'")
        except FileNotFoundError:
            print(f"Error: The file '{csv_file_path}' was not found.")
            return

        # --- Create relative time column ---
        if '__time' in df.columns:
            df['time'] = df['__time'] - df['__time'].iloc[0]
        elif 'time' not in df.columns:
             # If no time column, create a dummy one based on index for plotting
             df['time'] = df.index * 0.01 # Assuming 100Hz if no time is given

        # --- Handle potential missing velocity/acceleration data ---
        df[col_acc_g] = df[col_acc_g].fillna(0)
        df[col_vel_g] = df[col_vel_g].fillna(0)

        self.df = df

        # --- Parse filename for target_theta_g ---
        match = re.search(r'(\d+\.\d+)', self.current_filename)
        default_target = 0.3
        if match:
            try:
                self.target_theta_g = float(match.group(1))
                print(f"Auto-set target_theta_g from filename to: {self.target_theta_g}")
            except (ValueError, IndexError):
                self.target_theta_g = default_target
                print(f"Could not parse float from filename. Defaulting target_theta_g to: {default_target}")
        else:
            self.target_theta_g = default_target
            print(f"Filename does not contain a float. Defaulting target_theta_g to: {default_target}")

        self.text_target_theta.set_val(str(self.target_theta_g))

        if self.use_auto_initial_time:
            self.run_auto_initial_time_detection()
        else:
            self.recalculate_and_plot()

    def open_file_dialog(self, event):
        """Opens a file dialog to select a CSV file."""
        # We don't need to create and destroy a new Tk root window here.
        # Matplotlib's TkAgg backend already manages a root window, and creating
        # a new one and destroying it can interfere with the main event loop.
        filepath = filedialog.askopenfilename(
            initialdir=os.path.expanduser('~/wheel_loader_ws/results/csv/settle/'),
            title="Select a CSV file",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*"))
        )

        if filepath:
            self.load_file(filepath)
        else:
            print("File selection cancelled.")


def main():
    """
    Main function to load data, calculate mass, and plot results,
    simulating spreadsheet-like operations.
    """
    # --- Load Data ---
    initial_filename = 'down0.3_3.15_1'
    csv_file_path = os.path.expanduser(f'~/wheel_loader_ws/results/csv/settle/{initial_filename}.csv')

    try:
        df = pd.read_csv(csv_file_path)
        print(f"Successfully loaded '{csv_file_path}'")
    except FileNotFoundError:
        print(f"Error: The file '{csv_file_path}' was not found.")
        return

    # --- Check for required columns ---
    required_cols = [col_theta_g, col_pb, col_pr, col_acc_g, col_vel_g]
    if not all(col in df.columns for col in required_cols):
        print(f"Error: One or more required columns ({required_cols}) not found in the CSV file.")
        print(f"Available columns are: {df.columns.tolist()}")
        return

    # --- Create relative time column ---
    if '__time' in df.columns:
        df['time'] = df['__time'] - df['__time'].iloc[0]
    elif 'time' not in df.columns:
        # If no time column, create a dummy one based on index for plotting
        df['time'] = df.index * 0.01 # Assuming 100Hz if no time is given

    # --- Handle potential missing velocity/acceleration data ---
    # Assuming NaN means no movement, fill with 0.
    df[col_acc_g] = df[col_acc_g].fillna(0)
    df[col_vel_g] = df[col_vel_g].fillna(0)

    # --- Launch Interactive UI ---
    app = InteractiveTuner(df, initial_filename)
    plt.show()


if __name__ == '__main__':
    main()