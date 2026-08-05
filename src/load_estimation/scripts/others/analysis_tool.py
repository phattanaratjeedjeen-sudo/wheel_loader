#!/usr/bin/python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.widgets import CheckButtons, Button, TextBox
from tkinter import filedialog
import re
from scipy.optimize import curve_fit
from matplotlib import cm
import subprocess
import threading
import queue

# --- Constants ---
Lgh = 1.63
Lgi = np.sqrt(0.135**2 + 0.64**2)
Lag = 3.9294
IGO = np.pi - np.arctan2(0.64, 0.135)

# Cylinder areas
Ab = 0.020  # m^2, bottom cross-section area
Ar = 0.014  # m^2, piston cross-section area

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
        self.sensor_scale_factor = 1.0
        self.theta_g_offset = -0.62
        self.toPa_base = 40 * 10**6 / (2**15 - 1)
        self.use_compensation = False
        self.k1 = 900.0
        self.k2 = -550
        self.use_surf_fit = False
        self.C1 = -0.2353
        self.C2 = 1.0570e-03
        self.C3 = 4.9620e-01
        self.surf_fit_rhs = f"{self.C1} + {self.C2}*w + {self.C3}*theta_g"
        self.eq_deg1_rhs = self.surf_fit_rhs
        self.eq_best_rhs = self.surf_fit_rhs
        self.eq_best_degree = 1
        self.use_median_filter = False
        self.use_ema_filter = True
        self.use_pressure_offset = True
        self.ema_alpha = 0.001
        self.workspace_root = os.path.expanduser('~/wheel_loader_ws/results/csv')
        self.use_theta_g_ema_filter = True
        self.theta_g_ema_alpha = 0.1
        self.median_window = 100.0
        self.slope_time_interval = 0.1
        self.slope_threshold = 1.0
        self.use_slope_filter = True
        self.slope_ema_alpha = 0.001
        self.ts_param = 60.0
        self.settling_duration_req = 10.0
        self.duration_param = 1.0
        self.auto_set_theta_g = False
        self.fill_common_targets = False

        # Multi-target attributes
        self.target_theta_g_list = []
        self.analysis_targets = []  # List of dicts: {'target_g': g, 'initial_search_time': t}
        self.settling_results = []  # List of dicts for plotting results
        self.final_pressure_results = []  # List of dicts for final pressure results
        self.max_gih_annotations = [] # To hold vline and text artists for geometry figure
        self.surf_fit_export_data = [] # For exporting data for surface fitting
        self.ui_update_queue = queue.Queue()

        self.control_artists = []

        if self.auto_set_theta_g:
            match = re.search(r'(\d+\.\d+)', self.current_filename)
            default_target = 0.3
            if match:
                try:
                    self.target_theta_g_list = [float(match.group(1))]
                    print(f"Auto-set target_theta_g from filename to: {self.target_theta_g_list}")
                except (ValueError, IndexError):
                    self.target_theta_g_list = [default_target]
                    print(f"Could not parse float from filename. Defaulting target_theta_g to: {[default_target]}")
            else:
                self.target_theta_g_list = [default_target]
                print(f"Filename does not contain a float. Defaulting target_theta_g to: {[default_target]}")

        self.theta_g_threshold = 0.05 # used for auto-detect
        self.vel_g_threshold_auto = VEL_THRESHOLD
        self.acc_g_threshold_auto = ACC_THRESHOLD

        # --- Create Figure and Axes for plots---
        # --- Create a new Figure for Kinematics ---
        self.kinematics_fig, (self.ax_kin_theta, self.ax_kin_vel, self.ax_kin_acc) = plt.subplots(3, 1, num='Figure 1: Kinematics Analysis', figsize=(12, 8), sharex=True)
        self.kinematics_fig.subplots_adjust(left=0.08, bottom=0.05, right=0.95, top=0.90, hspace=0.3)

        # --- Create a new Figure for the time plot with an extra subplot for 'a' ---
        self.time_fig, (self.ax_time, self.ax_a) = plt.subplots(2, 1, num='Figure 2: Load & Geometry vs Time', figsize=(12, 8), sharex=True)
        self.time_fig.subplots_adjust(left=0.05, bottom=0.05, right=0.945, top=0.85, wspace=0.08, hspace=0.2)
        self.ax_a2 = self.ax_a.twinx()

        # --- Create a new Figure for pressure analysis ---
        self.pressure_fig = plt.figure(num='Figure 3: Pressure Analysis (pb)', figsize=(12, 8))
        gs = self.pressure_fig.add_gridspec(2, 2)
        self.ax_pressure_pb = self.pressure_fig.add_subplot(gs[0, 0])
        self.ax_theta_g_t = self.pressure_fig.add_subplot(gs[0, 1], sharex=self.ax_pressure_pb)
        self.ax_slope_pb = self.pressure_fig.add_subplot(gs[1, :], sharex=self.ax_pressure_pb)
        self.pressure_fig.subplots_adjust(left=0.05, bottom=0.05, right=0.945, top=0.90, wspace=0.08, hspace=0.2)

        # --- Create a new Figure for final pressure analysis ---
        self.final_pressure_fig, self.ax_final_pressure = plt.subplots(num='Figure 4: Final Pressure (pb)', figsize=(12, 8))
        self.final_pressure_fig.subplots_adjust(left=0.05, bottom=0.05, right=0.945, top=0.90, wspace=0.08, hspace=0.2)

        # --- Create a new Figure for pr pressure analysis ---
        self.pr_pressure_fig = plt.figure(num='Figure 5: Pressure Analysis (pr)', figsize=(12, 8))
        gs_pr = self.pr_pressure_fig.add_gridspec(2, 2)
        self.ax_pressure_pr = self.pr_pressure_fig.add_subplot(gs_pr[0, 0])
        self.ax_theta_g_t_pr = self.pr_pressure_fig.add_subplot(gs_pr[0, 1], sharex=self.ax_pressure_pr)
        self.ax_slope_pr = self.pr_pressure_fig.add_subplot(gs_pr[1, :], sharex=self.ax_pressure_pr)
        self.pr_pressure_fig.subplots_adjust(left=0.05, bottom=0.05, right=0.945, top=0.90, wspace=0.08, hspace=0.2)

        # --- Create a new Figure for pr final pressure analysis ---
        self.pr_final_pressure_fig, self.ax_pr_final_pressure = plt.subplots(num='Figure 7: Final Pressure (pr)', figsize=(12, 8))
        self.pr_final_pressure_fig.subplots_adjust(left=0.05, bottom=0.05, right=0.945, top=0.90, wspace=0.08, hspace=0.2)

        # --- Create a new Figure for Geometry ---
        self.geometry_fig, (self.ax_lih, self.ax_gih, self.ax_hio, self.ax_a_geom, self.ax_theta_g_geom) = plt.subplots(5, 1, num='Figure 6: Geometry', figsize=(10, 14), sharex=True)
        self.geometry_fig.subplots_adjust(left=0.1, bottom=0.05, right=0.95, top=0.92, hspace=0.4)

        # --- Create a new Figure for Validation ---
        self.validation_fig, (self.ax_rmse_s, self.ax_rmse_w) = plt.subplots(2, 1, num='Figure 8: Validation', figsize=(10, 8))
        self.validation_fig.subplots_adjust(left=0.08, bottom=0.08, right=0.95, top=0.92, hspace=0.4)

        # Add CheckButtons for the vertical line
        ax_check_v_line = self.geometry_fig.add_axes([0.75, 0.90, 0.2, 0.05]) # [left, bottom, width, height]
        self.check_v_line_geom = CheckButtons(ax_check_v_line, ['Show Max GIH'], actives=[True])
        self.check_v_line_geom.on_clicked(self.toggle_max_gih_line)
        
        # --- Create a separate Figure for controls ---
        self.control_fig = plt.figure("Tuning Panel", figsize=(7, 10.5))
        # We will manage adjustments manually to accommodate a scrollbar.
        # subplots_adjust will be called after all widgets are created.
        self.view_bottom = 0.02
        self.view_top = 0.98


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
        h_checkbox_triple = h_checkbox * 3

        # Gaps
        v_gap_widget = 0.008
        v_gap_section = 0.025
        h_gap_widget = 0.03

        # Proportions for common layouts (Checkbox width vs Textbox width)
        prop_checkbox = 0.50
        prop_widget = 1.0 - prop_checkbox - h_gap_widget
        prop_half = 0.5 - h_gap_widget / 2

        # --- Layout Helper Function ---
        def add_section_title(title):
            nonlocal y_cursor
            y_cursor -= v_gap_section
            text_artist = self.control_fig.text(left_margin, y_cursor, title, fontsize=10, weight='bold')
            self.control_artists.append(text_artist)
            y_cursor -= (h_widget + v_gap_widget) # Add space after title

        # --- Section 1: File & View ---
        add_section_title('File & View Controls')
        
        # Button on top
        y_cursor -= h_widget
        ax_select_btn = self.control_fig.add_axes([left_margin, y_cursor, content_width, h_widget])
        self.control_artists.append(ax_select_btn)
        y_cursor -= v_gap_widget

        # Checkboxes below in two columns for a more compact view
        fig_labels = ['Kinematics', 'Load(t)', 'Pressure(t) pb', 'Final Pressure pb', 'Pressure(t) pr', 'Geometry', 'Final Pressure pr', 'Validation']
        num_figs_col1 = 4
        self.fig_labels_col1 = fig_labels[:num_figs_col1]
        self.fig_labels_col2 = fig_labels[num_figs_col1:]
        
        row_h_checks = h_checkbox * num_figs_col1
        y_cursor -= row_h_checks
        
        w_half = content_width * prop_half
        ax_fig_vis1 = self.control_fig.add_axes([left_margin, y_cursor, w_half, row_h_checks])
        self.control_artists.append(ax_fig_vis1)
        ax_fig_vis2 = self.control_fig.add_axes([left_margin + w_half + h_gap_widget, y_cursor, w_half, row_h_checks])
        self.control_artists.append(ax_fig_vis2)
        y_cursor -= v_gap_widget

        # --- Section 2: Signal Filtering ---
        add_section_title('Pressure Signal Filtering')
        row_h = h_checkbox_triple
        y_cursor -= row_h
        widget_x = left_margin + content_width * prop_checkbox + h_gap_widget
        widget_w = content_width * prop_widget
        ax_filter_checks = self.control_fig.add_axes([left_margin, y_cursor, content_width * prop_checkbox, row_h])
        self.control_artists.append(ax_filter_checks)
        ax_median_win = self.control_fig.add_axes([widget_x, y_cursor + h_widget * 2, widget_w, h_widget])
        self.control_artists.append(ax_median_win)
        ax_ema_alpha = self.control_fig.add_axes([widget_x, y_cursor + h_widget, widget_w, h_widget])
        self.control_artists.append(ax_ema_alpha)
        y_cursor -= v_gap_widget

        # New widgets for theta_g EMA
        y_cursor -= h_checkbox
        ax_check_theta_g_filter = self.control_fig.add_axes([left_margin, y_cursor, content_width * prop_checkbox, h_widget+0.01])
        self.control_artists.append(ax_check_theta_g_filter)
        ax_theta_g_ema_alpha = self.control_fig.add_axes([widget_x, y_cursor, widget_w, h_widget])
        self.control_artists.append(ax_theta_g_ema_alpha)
        y_cursor -= v_gap_widget

        w_half = content_width * prop_half
        y_cursor -= h_widget
        ax_sensor_scale = self.control_fig.add_axes([left_margin, y_cursor, w_half, h_widget])
        self.control_artists.append(ax_sensor_scale)
        ax_theta_g_offset = self.control_fig.add_axes([left_margin + w_half + h_gap_widget, y_cursor, w_half, h_widget])
        self.control_artists.append(ax_theta_g_offset)
        y_cursor -= v_gap_widget

        # --- Section 3: Load Model Compensation ---
        add_section_title('Load Model Compensation')
        row_h = h_checkbox_double
        y_cursor -= row_h
        ax_check_comp1 = self.control_fig.add_axes([left_margin, y_cursor, content_width * prop_checkbox, row_h])
        self.control_artists.append(ax_check_comp1)
        ax_k1 = self.control_fig.add_axes([widget_x, y_cursor + h_widget, widget_w, h_widget])
        self.control_artists.append(ax_k1)
        ax_k2 = self.control_fig.add_axes([widget_x, y_cursor, widget_w, h_widget])
        self.control_artists.append(ax_k2)
        y_cursor -= v_gap_widget

        # --- Surf Fit Sub-section ---
        y_cursor -= h_widget
        w_third = (content_width - 2 * h_gap_widget) / 3
        ax_export_surf_fit_btn = self.control_fig.add_axes([left_margin, y_cursor, w_third, h_widget])
        self.control_artists.append(ax_export_surf_fit_btn)
        ax_find_surf_fit_btn = self.control_fig.add_axes([left_margin + w_third + h_gap_widget, y_cursor, w_third, h_widget])
        self.control_artists.append(ax_find_surf_fit_btn)
        ax_validate_btn = self.control_fig.add_axes([left_margin + 2 * (w_third + h_gap_widget), y_cursor, w_third, h_widget])
        self.control_artists.append(ax_validate_btn)
        y_cursor -= v_gap_widget

        # Add text display and apply button for Degree 1
        y_cursor -= h_widget
        ax_apply_deg1_btn = self.control_fig.add_axes([left_margin, y_cursor, prop_checkbox, h_widget])
        self.control_artists.append(ax_apply_deg1_btn)
        self.text_eq_deg1 = self.control_fig.text(left_margin + prop_checkbox + h_gap_widget, y_cursor + h_widget/2, "Eq Deg 1: (Click 'Find Equations' first)", va='center', ha='left', fontsize=8, wrap=True)
        self.control_artists.append(self.text_eq_deg1)
        y_cursor -= v_gap_widget

        # Add text display and apply button for Best Fit
        y_cursor -= h_widget
        ax_apply_best_btn = self.control_fig.add_axes([left_margin, y_cursor, prop_checkbox, h_widget])
        self.control_artists.append(ax_apply_best_btn)
        self.text_eq_best = self.control_fig.text(left_margin + prop_checkbox + h_gap_widget, y_cursor + h_widget/2, "Eq Best: (Click 'Find Equations' first)", va='center', ha='left', fontsize=8, wrap=True)
        self.control_artists.append(self.text_eq_best)
        y_cursor -= v_gap_widget

        # --- Section 4: Settling Time Analysis ---
        add_section_title('Settling Time Analysis')
        
        y_cursor -= h_checkbox
        ax_check_auto_theta = self.control_fig.add_axes([left_margin, y_cursor, content_width, h_checkbox+0.01])
        self.control_artists.append(ax_check_auto_theta)
        y_cursor -= v_gap_widget

        y_cursor -= h_checkbox
        ax_fill_targets = self.control_fig.add_axes([left_margin, y_cursor, content_width, h_checkbox+0.01])
        self.control_artists.append(ax_fill_targets)
        y_cursor -= v_gap_widget

        # Row for target theta list
        y_cursor -= h_widget
        ax_target_theta = self.control_fig.add_axes([left_margin, y_cursor, content_width, h_widget])
        self.control_artists.append(ax_target_theta)
        y_cursor -= v_gap_widget

        # Row for thresholds (3 widgets)
        w_third = (content_width - 2 * h_gap_widget) / 3
        y_cursor -= h_widget
        ax_theta_thresh = self.control_fig.add_axes([left_margin, y_cursor, w_third, h_widget])
        self.control_artists.append(ax_theta_thresh)
        ax_vel_thresh_auto = self.control_fig.add_axes([left_margin + w_third + h_gap_widget, y_cursor, w_third, h_widget])
        self.control_artists.append(ax_vel_thresh_auto)
        ax_acc_thresh_auto = self.control_fig.add_axes([left_margin + 2 * (w_third + h_gap_widget), y_cursor, w_third, h_widget])
        self.control_artists.append(ax_acc_thresh_auto)
        y_cursor -= v_gap_widget

        # Row for slope/settling parameters (3 widgets)
        y_cursor -= h_widget
        ax_settling_dur_req = self.control_fig.add_axes([left_margin, y_cursor, w_third, h_widget])
        ax_slope_interval = self.control_fig.add_axes([left_margin + w_third + h_gap_widget, y_cursor, w_third, h_widget])
        ax_slope_thresh = self.control_fig.add_axes([left_margin + 2 * (w_third + h_gap_widget), y_cursor, w_third, h_widget])
        self.control_artists.append(ax_settling_dur_req)
        self.control_artists.append(ax_slope_interval)
        self.control_artists.append(ax_slope_thresh)
        y_cursor -= v_gap_widget

        # Row for slope filter checkbox and its textbox
        row_h = h_checkbox_double
        y_cursor -= row_h
        ax_check_slope_filter = self.control_fig.add_axes([left_margin, y_cursor, content_width * prop_checkbox, row_h])
        self.control_artists.append(ax_check_slope_filter)
        ax_slope_ema_alpha = self.control_fig.add_axes([widget_x, y_cursor + row_h/2 - h_widget/2, widget_w, h_widget])
        self.control_artists.append(ax_slope_ema_alpha)
        y_cursor -= v_gap_widget

        # --- Section 5: Final Pressure Analysis ---
        add_section_title('Final Pressure Analysis')
        y_cursor -= h_widget
        textbox_x_final = left_margin + content_width * 0.1
        textbox_w_final = content_width * (prop_half - 0.1)
        ax_ts_param = self.control_fig.add_axes([textbox_x_final, y_cursor, textbox_w_final, h_widget])
        self.control_artists.append(ax_ts_param)
        ax_duration_param = self.control_fig.add_axes([textbox_x_final + textbox_w_final + h_gap_widget, y_cursor, textbox_w_final, h_widget])
        self.control_artists.append(ax_duration_param)
        y_cursor -= v_gap_widget

        # --- Instantiate Widgets (in section order) ---
        # Section 1: File & View
        self.btn_select_file = Button(ax_select_btn, 'Select CSV File')
        self.check_figs1 = CheckButtons(ax_fig_vis1, self.fig_labels_col1, actives=[False]*len(self.fig_labels_col1))
        self.check_figs2 = CheckButtons(ax_fig_vis2, self.fig_labels_col2, actives=[False]*len(self.fig_labels_col2))

        # Section 2: Signal Filtering
        self.check_filters = CheckButtons(ax_filter_checks, ['Median', 'EMA', 'Offset'], actives=[self.use_median_filter, self.use_ema_filter, self.use_pressure_offset])
        self.text_median_win = TextBox(ax_median_win, 'Median Win', initial=str(self.median_window))
        self.text_ema_alpha = TextBox(ax_ema_alpha, 'EMA Alpha', initial=str(self.ema_alpha))
        self.check_theta_g_filter = CheckButtons(ax_check_theta_g_filter, ['EMA on θg'], [self.use_theta_g_ema_filter])
        self.text_theta_g_ema_alpha = TextBox(ax_theta_g_ema_alpha, 'θg EMA α', initial=str(self.theta_g_ema_alpha))
        self.text_sensor_scale = TextBox(ax_sensor_scale, 'Sensor Scale', initial=str(self.sensor_scale_factor))
        self.text_theta_g_offset = TextBox(ax_theta_g_offset, 'θg Offset (rad)', initial=str(self.theta_g_offset))

        # Section 3: Load Model Compensation
        self.check_comp = CheckButtons(ax_check_comp1, ['Compensator 1', 'Surf Fit'], [self.use_compensation, self.use_surf_fit])
        self.text_k1 = TextBox(ax_k1, 'k1', initial=str(self.k1))
        self.text_k2 = TextBox(ax_k2, 'k2', initial=str(self.k2))
        self.btn_export_surf_fit = Button(ax_export_surf_fit_btn, 'Export for Surf Fit')
        self.btn_find_surf_fit = Button(ax_find_surf_fit_btn, 'Find Equations')
        self.btn_validate = Button(ax_validate_btn, 'Validate')
        self.btn_apply_deg1 = Button(ax_apply_deg1_btn, 'Apply Degree 1')
        self.btn_apply_best = Button(ax_apply_best_btn, 'Apply Best Fit')
        self.btn_apply_deg1.ax.set_visible(False) # Hide until equations are found
        self.btn_apply_best.ax.set_visible(False)

        # Section 4: Settling Time Analysis
        self.check_auto_theta = CheckButtons(ax_check_auto_theta, ['Auto-set θg from filename'], [self.auto_set_theta_g])
        self.check_fill_targets = CheckButtons(ax_fill_targets, ['Fill Common Targets'], [self.fill_common_targets])
        self.text_target_theta = TextBox(ax_target_theta, 'Target θg (csv)', initial=', '.join(map(str, self.target_theta_g_list)))
        self.text_theta_thresh = TextBox(ax_theta_thresh, 'θg Thresh', initial=str(self.theta_g_threshold))
        self.text_vel_thresh = TextBox(ax_vel_thresh_auto, 'Vel Thresh', initial=str(self.vel_g_threshold_auto))
        self.text_acc_thresh = TextBox(ax_acc_thresh_auto, 'Acc Thresh', initial=str(self.acc_g_threshold_auto))
        self.text_settling_dur_req = TextBox(ax_settling_dur_req, 'Stable Dur', initial=str(self.settling_duration_req))
        self.text_slope_interval = TextBox(ax_slope_interval, 'Slope Interval (s)', initial=str(self.slope_time_interval))
        self.text_slope_thresh = TextBox(ax_slope_thresh, 'Slope Thresh', initial=str(self.slope_threshold))
        self.check_slope_filter = CheckButtons(ax_check_slope_filter, ['Filter Slope (EMA)'], [self.use_slope_filter])
        self.text_slope_ema_alpha = TextBox(ax_slope_ema_alpha, 'Slope EMA α', initial=str(self.slope_ema_alpha))

        # Section 5: Final Pressure Analysis
        self.text_ts_param = TextBox(ax_ts_param, 'Ts (sec)', initial=str(self.ts_param))
        self.text_duration_param = TextBox(ax_duration_param, 'Duration (sec)', initial=str(self.duration_param))

        # --- Connect UI Events ---
        self.btn_select_file.on_clicked(self.open_file_dialog)
        self.text_k1.on_submit(self.update_params)
        self.text_k2.on_submit(self.update_params)
        self.check_comp.on_clicked(self.toggle_compensation)
        self.check_figs1.on_clicked(self.toggle_figure_visibility)
        self.check_figs2.on_clicked(self.toggle_figure_visibility)
        self.check_filters.on_clicked(self.toggle_filters)
        self.text_ema_alpha.on_submit(self.update_filter_params)
        self.check_theta_g_filter.on_clicked(self.toggle_theta_g_filter)
        self.text_theta_g_ema_alpha.on_submit(self.update_theta_g_filter_params)
        self.text_median_win.on_submit(self.update_filter_params)
        self.text_slope_interval.on_submit(self.update_slope_params)
        self.text_slope_thresh.on_submit(self.update_settling_params)
        self.check_auto_theta.on_clicked(self.toggle_auto_theta)
        self.check_fill_targets.on_clicked(self.toggle_fill_targets)
        self.text_target_theta.on_submit(self.update_auto_time_params)
        self.text_theta_thresh.on_submit(self.update_auto_time_params)
        self.text_vel_thresh.on_submit(self.update_auto_time_params)
        self.text_acc_thresh.on_submit(self.update_auto_time_params)
        self.text_settling_dur_req.on_submit(self.update_settling_params)
        self.check_slope_filter.on_clicked(self.toggle_slope_filter)
        self.text_slope_ema_alpha.on_submit(self.update_slope_filter_params)
        self.text_ts_param.on_submit(self.update_final_pressure_params)
        self.text_duration_param.on_submit(self.update_final_pressure_params)
        self.text_sensor_scale.on_submit(self.update_sensor_scale_factor)
        self.btn_export_surf_fit.on_clicked(self.export_for_surf_fit)
        self.btn_find_surf_fit.on_clicked(self.find_surf_fit_equations)
        self.btn_apply_deg1.on_clicked(self.apply_deg1_equation)
        self.btn_validate.on_clicked(self.validate_and_export)
        self.btn_apply_best.on_clicked(self.apply_best_equation)
        self.text_theta_g_offset.on_submit(self.update_theta_g_offset)

        # --- Finalize Control UI ---
        self.control_fig.subplots_adjust(left=0.05, right=0.95, top=self.view_top, bottom=self.view_bottom)
        self.control_fig.canvas.mpl_connect('scroll_event', self.on_scroll)

        # --- Initial Calculation and Plot ---
        self.recalculate_and_plot()

        # Hide all plot figures by default on startup
        self.toggle_figure_visibility(None)

        # Start the queue checker to listen for UI updates from background threads
        self.process_ui_updates()

    def on_scroll(self, event):
        """Handles mouse wheel scrolling on the control panel."""
        all_y = []
        for artist in self.control_artists:
            if isinstance(artist, plt.Axes):
                pos = artist.get_position()
                all_y.append(pos.y0)
                all_y.append(pos.y1)
            elif isinstance(artist, plt.Text):
                all_y.append(artist.get_position()[1])

        if not all_y:
            return

        min_y, max_y = min(all_y), max(all_y)

        # Mouse wheel up (step=1) should move content down (dy<0)
        # Mouse wheel down (step=-1) should move content up (dy>0)
        dy = -event.step * 0.05

        # dy > 0: content moves UP. Stop when bottom of content reaches bottom of view.
        if dy > 0:
            scroll_up_space = self.view_bottom - min_y
            if scroll_up_space <= 0:
                return
            dy = min(dy, scroll_up_space)

        # dy < 0: content moves DOWN. Stop when top of content reaches top of view.
        elif dy < 0:
            scroll_down_space = self.view_top - max_y
            if scroll_down_space >= 0:
                return
            dy = max(dy, scroll_down_space)

        for artist in self.control_artists:
            if isinstance(artist, plt.Axes):
                pos = artist.get_position()
                artist.set_position([pos.x0, pos.y0 + dy, pos.width, pos.height])
            elif isinstance(artist, plt.Text):
                x, y = artist.get_position()
                artist.set_position((x, y + dy))

        self.control_fig.canvas.draw_idle()

    def calculate_mass(self, theta_a, Fc):
        Lih = np.sqrt(Lgh**2 + Lgi**2 - 2 * Lgh * Lgi * np.cos(theta_a + IGO))
        GIH_cos = (Lih**2 + Lgi**2 - Lgh**2) / (2 * Lih * Lgi)
        GIH_sin = (Lgh / Lih) * np.sin(theta_a + IGO)
        GIH = np.arctan2(GIH_sin, GIH_cos)
        HIO = np.pi - IGO - GIH
        a = np.sin(HIO) - np.cos(HIO)*np.tan(theta_a)
        
        simple_lever = ((Fc * Lgh / Lag) * a) / 9.807

        w = simple_lever
        if self.use_compensation:
            compensator = self.k1*theta_a + self.k2
            w += compensator

        return w, simple_lever

    def calculate_mass_surf_fit(self, theta_a, Fc, theta_g_raw):
        # 1. Calculate simple_lever, which depends on theta_a (with offset)
        Lih = np.sqrt(Lgh**2 + Lgi**2 - 2 * Lgh * Lgi * np.cos(theta_a + IGO))
        GIH_cos = (Lih**2 + Lgi**2 - Lgh**2) / (2 * Lih * Lgi)
        GIH_sin = (Lgh / Lih) * np.sin(theta_a + IGO)
        GIH = np.arctan2(GIH_sin, GIH_cos)
        HIO = np.pi - IGO - GIH
        a = np.sin(HIO) - np.cos(HIO)*np.tan(theta_a)

        simple_lever = ((Fc * Lgh / Lag) * a) / 9.807

        # 2. Apply surface fit equation using eval
        # The equation from surface_fit.py uses 'w' and 'theta_g' as variable names
        scope = {
            'w': simple_lever,
            'theta_g': theta_g_raw,
            'np': np # For safety if any numpy functions are in the string
        }
        
        # The self.surf_fit_rhs string is like "-1.3248 + 1.0649e-03 * w + 9.7453e-01 * theta_g"
        load_in_tons = eval(self.surf_fit_rhs, {'np': np}, scope)
        return load_in_tons * 1000, simple_lever

    def recalculate_and_plot(self):
        # --- Define helper functions ---
        # Exponential function for curve fitting
        def exp_func(x, a, b, c):
            return a * np.exp(b * x) + c

        # --- Perform Calculation ---
        theta_g_raw = self.df[col_theta_g]
        theta_a = theta_g_raw + self.theta_g_offset

        # --- Apply EMA to theta_a and recalculate derivatives if enabled ---
        vel_g = self.df[col_vel_g]
        acc_g = self.df[col_acc_g]
        if self.use_theta_g_ema_filter:
            theta_a = theta_a.ewm(alpha=self.theta_g_ema_alpha, adjust=False).mean()
            # Recalculate derivatives from filtered theta_a
            time_diff = self.df['time'].diff().replace(0, 1e-9)
            vel_g = theta_a.diff() / time_diff
            acc_g = vel_g.diff() / time_diff

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

        # --- Apply Pressure Offset ---
        if self.use_pressure_offset:
            pb_offset = 1699 * theta_g_raw + 4023
            pr_offset = 413.3
            pb_filtered -= pb_offset
            pr_filtered -= pr_offset
            # Clip at zero to prevent negative pressures
            pb_filtered = pb_filtered.clip(lower=0)
            pr_filtered = pr_filtered.clip(lower=0)

        # --- Calculate Force ---
        # Calculate the final Pa conversion factor, which now includes the scale factor
        toPa = self.toPa_base * self.sensor_scale_factor
        Fc = 2 * (Ab * pb_filtered - Ar * pr_filtered) * toPa

        self.df[col_w_calculated], _ = self.calculate_mass(theta_a, Fc)

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
        slope_pr = pr_filtered.diff(periods=samples_per_interval) / time_delta

        # --- Filter the slope if enabled ---
        slope_to_analyze_pb = slope_pb.copy()
        if self.use_slope_filter:
            slope_to_analyze_pb = slope_to_analyze_pb.ewm(alpha=self.slope_ema_alpha, adjust=False).mean()

        slope_to_analyze_pr = slope_pr.copy()
        if self.use_slope_filter:
            slope_to_analyze_pr = slope_to_analyze_pr.ewm(alpha=self.slope_ema_alpha, adjust=False).mean()

        # --- Clear and Redraw Plots ---
        self.ax_time.clear()
        self.ax_kin_theta.clear()
        self.ax_kin_vel.clear()
        self.ax_kin_acc.clear()
        self.ax_a.clear()
        self.ax_a2.clear()
        self.ax_pressure_pb.clear()
        self.ax_theta_g_t.clear()
        self.ax_slope_pb.clear()
        self.ax_final_pressure.clear()
        self.ax_pressure_pr.clear()
        self.ax_theta_g_t_pr.clear()
        self.ax_slope_pr.clear()
        self.ax_pr_final_pressure.clear()
        self.ax_lih.clear()
        self.ax_gih.clear()
        self.ax_hio.clear()
        self.ax_a_geom.clear()
        self.ax_theta_g_geom.clear()

        # --- Clear previous multi-target results ---
        self.max_gih_annotations = [] # Just clear the list, artists are cleared with axes
        self.settling_results = []
        self.final_pressure_results = []
        self.surf_fit_export_data = [] # Clear export data on each replot
        colors = cm.get_cmap('viridis')(np.linspace(0, 1, max(1, len(self.analysis_targets))))

        # --- Loop through each detected target for analysis ---
        for i, target_info in enumerate(self.analysis_targets):
            initial_search_time = target_info['initial_search_time']
            target_g = target_info['target_g']
            color = colors[i]

            # --- Find Settling Time (for this target) ---
            settling_time_pb, settling_duration_pb = None, None
            settling_time_pr, settling_duration_pr = None, None

            avg_dt_settle = self.df['time'].diff().mean()
            if pd.isna(avg_dt_settle) or avg_dt_settle < 1e-6:
                stable_samples = int(self.settling_duration_req / 0.01)
            else:
                stable_samples = max(1, int(round(self.settling_duration_req / avg_dt_settle)))

            start_search_idx = (self.df['time'] - initial_search_time).abs().idxmin()
            theta_g_start = float(self.df.at[start_search_idx, col_theta_g])
            theta_g_settle_threshold = 0.1
            theta_g_stable_mask = (self.df[col_theta_g] - theta_g_start).abs() <= theta_g_settle_threshold

            # Analysis for pb
            slope_stable_mask_pb = slope_to_analyze_pb.abs() <= self.slope_threshold
            is_stable_pb = slope_stable_mask_pb & theta_g_stable_mask
            stable_window_sum_pb = is_stable_pb.rolling(window=stable_samples).sum()
            stable_window_end_indices = stable_window_sum_pb[stable_window_sum_pb >= stable_samples].index
            stable_window_start_indices = stable_window_end_indices - stable_samples + 1
            valid_start_indices = stable_window_start_indices[stable_window_start_indices >= start_search_idx]

            if not valid_start_indices.empty:
                settling_iloc = valid_start_indices[0]
                settling_time_pb = self.df.iloc[settling_iloc]['time']
                if settling_time_pb is not None:
                    settling_duration_pb = max(0.0, float(settling_time_pb - initial_search_time))

            # Analysis for pr
            slope_stable_mask_pr = slope_to_analyze_pr.abs() <= self.slope_threshold
            is_stable_pr = slope_stable_mask_pr & theta_g_stable_mask
            stable_window_sum_pr = is_stable_pr.rolling(window=stable_samples).sum()
            stable_window_end_indices_pr = stable_window_sum_pr[stable_window_sum_pr >= stable_samples].index
            stable_window_start_indices_pr = stable_window_end_indices_pr - stable_samples + 1
            valid_start_indices_pr = stable_window_start_indices_pr[stable_window_start_indices_pr >= start_search_idx]

            if not valid_start_indices_pr.empty:
                settling_iloc_pr = valid_start_indices_pr[0]
                settling_time_pr = self.df.iloc[settling_iloc_pr]['time']
                if settling_time_pr is not None:
                    settling_duration_pr = max(0.0, float(settling_time_pr - initial_search_time))

            self.settling_results.append({
                'target_g': target_g,
                'initial_search_time': initial_search_time,
                'settling_time_pb': settling_time_pb,
                'settling_duration_pb': settling_duration_pb,
                'settling_time_pr': settling_time_pr,
                'settling_duration_pr': settling_duration_pr,
                'color': color
            })

            # --- Final Pressure Analysis (for this target) ---
            analysis_start_time = initial_search_time
            analysis_end_time = analysis_start_time + self.duration_param
            analysis_mask = (self.df['time'] >= analysis_start_time) & (self.df['time'] <= analysis_end_time)
            analysis_df = self.df[analysis_mask].copy()

            median_pbf, median_prf = None, None
            popt_pb, popt_pr = None, None
            r_squared_pb, r_squared_pr = None, None
            t_pbf, pbf_values, t_prf, prf_values = [], [], [], []

            if not analysis_df.empty and len(analysis_df) > 1:
                analysis_df['t_shifted'] = analysis_df['time'] - analysis_start_time
                t_np = analysis_df['t_shifted'].to_numpy(dtype=float)
                tau = 0.25 * self.ts_param
                
                # For pb
                pb_window = pb_filtered[analysis_mask]
                pbi = pb_window.iloc[0]
                pb_window_np = pb_window.to_numpy(dtype=float)

                # For pr
                pr_window = pr_filtered[analysis_mask]
                pri = pr_window.iloc[0]
                pr_window_np = pr_window.to_numpy(dtype=float)

                with np.errstate(divide='ignore', invalid='ignore'): # Suppress div by zero on first element
                    exp_term = np.exp(-t_np / tau)
                    pbf_float = (pb_window_np - pbi * exp_term) / (1 - exp_term)
                    prf_float = (pr_window_np - pri * exp_term) / (1 - exp_term)

                pbf_values = pbf_float[1:]
                t_pbf = t_np[1:]
                if len(pbf_values) > 0: median_pbf = np.median(pbf_values)

                prf_values = prf_float[1:]
                t_prf = t_np[1:]
                if len(prf_values) > 0: median_prf = np.median(prf_values)

                try:
                    p0 = (pb_window_np[0] - pb_window_np[-1], -0.1, pb_window_np[-1])
                    popt_pb, _ = curve_fit(exp_func, t_np, pb_window_np, p0=p0, maxfev=5000, bounds=([-np.inf, -np.inf, 0], [np.inf, 0, np.inf]))
                    ss_total_pb = np.sum((pb_window_np - np.mean(pb_window_np))**2)
                    if ss_total_pb > 1e-9: # Avoid division by zero for constant data
                        residuals = pb_window_np - exp_func(t_np, *popt_pb)
                        ss_res_pb = np.sum(residuals**2)
                        r_squared_pb = 1 - (ss_res_pb / ss_total_pb)
                except (RuntimeError, ValueError): pass

                try:
                    p0 = (pr_window_np[0] - pr_window_np[-1], -0.1, pr_window_np[-1])
                    popt_pr, _ = curve_fit(exp_func, t_np, pr_window_np, p0=p0, maxfev=5000, bounds=([-np.inf, -np.inf, 0], [np.inf, 0, np.inf]))
                    ss_total_pr = np.sum((pr_window_np - np.mean(pr_window_np))**2)
                    if ss_total_pr > 1e-9: # Avoid division by zero for constant data
                        residuals = pr_window_np - exp_func(t_np, *popt_pr)
                        ss_res_pr = np.sum(residuals**2)
                        r_squared_pr = 1 - (ss_res_pr / ss_total_pr)
                except (RuntimeError, ValueError): pass

            self.final_pressure_results.append({
                'target_g': target_g,
                'initial_search_time': initial_search_time,
                'median_pbf': median_pbf,
                'median_prf': median_prf,
                't_pbf': t_pbf,
                'pbf_values': pbf_values,
                't_prf': t_prf,
                'prf_values': prf_values,
                'fit_params_pb': popt_pb,
                'fit_params_pr': popt_pr,
                'r_squared_pb': r_squared_pb,
                'r_squared_pr': r_squared_pr,
                'color': color
            })

        # --- Plot for Kinematics Analysis ---
        self.kinematics_fig.suptitle(f'Figure 1: Kinematics Analysis for "{self.current_filename}"')
        
        # Theta_g plot
        self.ax_kin_theta.plot(self.df['time'], theta_g_raw, label='Raw θg (from CSV)', color='cyan', alpha=0.7)
        self.ax_kin_theta.plot(self.df['time'], theta_a, label='Processed θa (with offset/filter)', color='blue')
        self.ax_kin_theta.set_title('Boom Angle')
        self.ax_kin_theta.set_ylabel('Angle (rad)')
        self.ax_kin_theta.grid(True)
        self.ax_kin_theta.legend()

        # Velocity plot
        self.ax_kin_vel.plot(self.df['time'], self.df[col_vel_g], label='Raw vel_g', color='orange', alpha=0.7)
        self.ax_kin_vel.plot(self.df['time'], vel_g, label='Processed vel_g', color='red')
        self.ax_kin_vel.set_title('Boom Angular Velocity (vel_g)')
        self.ax_kin_vel.set_ylabel('Velocity (rad/s)')
        self.ax_kin_vel.grid(True)
        self.ax_kin_vel.legend()

        # Acceleration plot
        self.ax_kin_acc.plot(self.df['time'], self.df[col_acc_g], label='Raw acc_g', color='lightgreen', alpha=0.7)
        self.ax_kin_acc.plot(self.df['time'], acc_g, label='Processed acc_g', color='green')
        self.ax_kin_acc.set_title('Boom Angular Acceleration (acc_g)')
        self.ax_kin_acc.set_ylabel('Acceleration (rad/s²)')
        self.ax_kin_acc.set_xlabel('Time (s)')
        self.ax_kin_acc.grid(True)
        self.ax_kin_acc.legend()

        # --- Plot for Pressure Analysis in a separate window ---
        self.pressure_fig.suptitle(f'Figure 3: Pressure Analysis (pb) for "{self.current_filename}"')
        if 'time' in self.df.columns:
            # --- Top-left: Raw vs Filtered Pressure (pb) ---
            self.ax_pressure_pb.plot(self.df['time'], self.df[col_pb], label='Raw pb', color='cyan', alpha=0.7)
            self.ax_pressure_pb.plot(self.df['time'], pb_filtered, label=f'Filtered pb ({self.applied_filters_str})', color='blue')

            self.ax_pressure_pb.set_title('Raw vs. Filtered Pressure (pb)')
            self.ax_pressure_pb.set_ylabel('Pressure (ADC value)')
            self.ax_pressure_pb.legend(fontsize='small')
            self.ax_pressure_pb.grid(True)

            # --- Top-right: Boom Angle (theta_g) vs Time ---
            self.ax_theta_g_t.plot(self.df['time'], theta_g_raw, label='θg (raw)', color='purple')
            self.ax_theta_g_t.set_title('Boom Angle (θg) vs. Time')
            self.ax_theta_g_t.set_ylabel('θg (rad)')
            self.ax_theta_g_t.grid(True)

            # --- Add multi-target lines to pressure plots ---
            for res in self.settling_results:
                color = res['color']
                initial_search_time = res['initial_search_time']
                settling_time_pb = res['settling_time_pb']
                label_prefix = f"Tgt {res['target_g']:.2f}"

                # Add lines to pb pressure plot
                self.ax_pressure_pb.axvline(x=initial_search_time, color=color, linestyle=':', lw=2, label=f'Start {label_prefix}')
                if settling_time_pb is not None:
                    self.ax_pressure_pb.axvline(x=settling_time_pb, color=color, linestyle='--', lw=2, label=f'Settle {label_prefix}')

                # Add text annotations for time and pressure
                start_idx = (self.df['time'] - initial_search_time).abs().idxmin()
                pressure_at_start = pb_filtered.at[start_idx]
                time_at_start = self.df.at[start_idx, 'time']
                self.ax_pressure_pb.text(time_at_start, pressure_at_start, f' ({time_at_start:.1f}s, {pressure_at_start:.1f})', color=color, ha='left', va='bottom', fontsize=9)

                if settling_time_pb is not None:
                    settle_idx = (self.df['time'] - settling_time_pb).abs().idxmin()
                    pressure_at_settle = pb_filtered.at[settle_idx]
                    time_at_settle = self.df.at[settle_idx, 'time']
                    self.ax_pressure_pb.text(time_at_settle, pressure_at_settle, f' ({time_at_settle:.1f}s, {pressure_at_settle:.1f})', color=color, ha='left', va='top', fontsize=9)

                # Add lines to theta_g plot
                self.ax_theta_g_t.axvline(x=initial_search_time, color=color, linestyle=':', lw=2)
                if settling_time_pb is not None:
                    self.ax_theta_g_t.axvline(x=settling_time_pb, color=color, linestyle='--', lw=2)

                # Add lines to slope plot
                self.ax_slope_pb.axvline(x=initial_search_time, color=color, linestyle=':', lw=2)
                if settling_time_pb is not None:
                    self.ax_slope_pb.axvline(x=settling_time_pb, color=color, linestyle='--', lw=2)

            # --- Bottom row: Slope of pb ---
            title_slope_pb = 'Slope of Filtered pb (for Settling Time)'
            if self.use_slope_filter:
                self.ax_slope_pb.plot(self.df['time'], slope_pb, label='Unfiltered Slope', color='blue', alpha=0.4)
                self.ax_slope_pb.plot(self.df['time'], slope_to_analyze_pb, label='Filtered Slope (EMA)', color='red')
            else:
                self.ax_slope_pb.plot(self.df['time'], slope_pb, label='Slope pb', color='blue')

            self.ax_slope_pb.axhline(y=self.slope_threshold, color='gray', linestyle='--', lw=1)
            self.ax_slope_pb.axhline(y=-self.slope_threshold, color='gray', linestyle='--', lw=1, label=f'Threshold ({self.slope_threshold:.1f})')

            settle_durations_pb = [f"Tgt {r['target_g']:.2f}: {r['settling_duration_pb']:.2f}s" for r in self.settling_results if r['settling_duration_pb'] is not None]
            if settle_durations_pb:
                title_slope_pb += '\nTime to Settle: ' + ', '.join(settle_durations_pb)
            
            self.ax_slope_pb.set_title(title_slope_pb)
            self.ax_slope_pb.set_ylabel('Pressure Slope')
            self.ax_slope_pb.set_xlabel('Time (s)')
            self.ax_slope_pb.legend(fontsize='small')
            self.ax_slope_pb.grid(True)

        # --- Plot for pr Pressure Analysis in a separate window ---
        self.pr_pressure_fig.suptitle(f'Figure 5: Pressure Analysis (pr) for "{self.current_filename}"')
        if 'time' in self.df.columns:
            # --- Top-left: Raw vs Filtered Pressure (pr) ---
            self.ax_pressure_pr.plot(self.df['time'], self.df[col_pr], label='Raw pr', color='sandybrown', alpha=0.7)
            self.ax_pressure_pr.plot(self.df['time'], pr_filtered, label=f'Filtered pr ({self.applied_filters_str})', color='red')
            self.ax_pressure_pr.set_title('Raw vs. Filtered Pressure (pr)')
            self.ax_pressure_pr.set_ylabel('Pressure (ADC value)')
            self.ax_pressure_pr.legend(fontsize='small')
            self.ax_pressure_pr.grid(True)

            # --- Top-right: Boom Angle (theta_g) vs Time ---
            self.ax_theta_g_t_pr.plot(self.df['time'], theta_g_raw, label='θg (raw)', color='purple')
            self.ax_theta_g_t_pr.set_title('Boom Angle (θg) vs. Time')
            self.ax_theta_g_t_pr.set_ylabel('θg (rad)')
            self.ax_theta_g_t_pr.grid(True)

            # --- Add multi-target lines to pr pressure plots ---
            for res in self.settling_results:
                color = res['color']
                initial_search_time = res['initial_search_time']
                settling_time_pr = res['settling_time_pr']
                label_prefix = f"Tgt {res['target_g']:.2f}"

                self.ax_pressure_pr.axvline(x=initial_search_time, color=color, linestyle=':', lw=2, label=f'Start {label_prefix}')
                if settling_time_pr is not None:
                    self.ax_pressure_pr.axvline(x=settling_time_pr, color=color, linestyle='--', lw=2, label=f'Settle {label_prefix}')

                # Add text annotations for time and pressure
                start_idx = (self.df['time'] - initial_search_time).abs().idxmin()
                pressure_at_start = pr_filtered.at[start_idx]
                time_at_start = self.df.at[start_idx, 'time']
                self.ax_pressure_pr.text(time_at_start, pressure_at_start, f' ({time_at_start:.1f}s, {pressure_at_start:.1f})', color=color, ha='left', va='bottom', fontsize=9)

                if settling_time_pr is not None:
                    settle_idx = (self.df['time'] - settling_time_pr).abs().idxmin()
                    pressure_at_settle = pr_filtered.at[settle_idx]
                    time_at_settle = self.df.at[settle_idx, 'time']
                    self.ax_pressure_pr.text(time_at_settle, pressure_at_settle, f' ({time_at_settle:.1f}s, {pressure_at_settle:.1f})', color=color, ha='left', va='top', fontsize=9)

                self.ax_theta_g_t_pr.axvline(x=initial_search_time, color=color, linestyle=':', lw=2)
                if settling_time_pr is not None:
                    self.ax_theta_g_t_pr.axvline(x=settling_time_pr, color=color, linestyle='--', lw=2)

                self.ax_slope_pr.axvline(x=initial_search_time, color=color, linestyle=':', lw=2)
                if settling_time_pr is not None:
                    self.ax_slope_pr.axvline(x=settling_time_pr, color=color, linestyle='--', lw=2)

            # --- Bottom row: Slope of pr ---
            title_slope_pr = 'Slope of Filtered pr (for Settling Time)'
            if self.use_slope_filter:
                self.ax_slope_pr.plot(self.df['time'], slope_pr, label='Unfiltered Slope', color='red', alpha=0.4)
                self.ax_slope_pr.plot(self.df['time'], slope_to_analyze_pr, label='Filtered Slope (EMA)', color='darkred')
            else:
                self.ax_slope_pr.plot(self.df['time'], slope_pr, label='Slope pr', color='red')

            self.ax_slope_pr.axhline(y=self.slope_threshold, color='gray', linestyle='--', lw=1)
            self.ax_slope_pr.axhline(y=-self.slope_threshold, color='gray', linestyle='--', lw=1, label=f'Threshold ({self.slope_threshold:.1f})')

            settle_durations_pr = [f"Tgt {r['target_g']:.2f}: {r['settling_duration_pr']:.2f}s" for r in self.settling_results if r['settling_duration_pr'] is not None]
            if settle_durations_pr:
                title_slope_pr += '\nTime to Settle: ' + ', '.join(settle_durations_pr)
            
            self.ax_slope_pr.set_title(title_slope_pr)
            self.ax_slope_pr.set_ylabel('Pressure Slope')
            self.ax_slope_pr.set_xlabel('Time (s)')
            self.ax_slope_pr.legend(fontsize='small')
            self.ax_slope_pr.grid(True)

        # --- Plot for Final Pressure Analysis in a separate window ---
        # Clear axes first
        self.ax_final_pressure.clear()
        self.ax_pr_final_pressure.clear()

        self.final_pressure_fig.suptitle(f'Figure 4: Final Pressure Analysis (pb) for "{self.current_filename}"')
        self.pr_final_pressure_fig.suptitle(f'Figure 7: Final Pressure Analysis (pr) for "{self.current_filename}"')

        for res in self.final_pressure_results:
            color = res['color']
            target_g = res['target_g']
            label_prefix = f"Tgt {target_g:.2f}"

            # Plot for pb
            if res['median_pbf'] is not None:
                self.ax_final_pressure.plot(res['t_pbf'], res['pbf_values'], marker='o', linestyle='-', color=color, label=f'pbf ({label_prefix}): {res["median_pbf"]:.1f}')
                if res['fit_params_pb'] is not None:
                    final_pressure_fit = res['fit_params_pb'][2]
                    self.ax_final_pressure.axhline(y=final_pressure_fit, color=color, linestyle='--', label=f'Fit Final P ({label_prefix}): {final_pressure_fit:.2f}')

            # Plot for pr
            if res['median_prf'] is not None:
                self.ax_pr_final_pressure.plot(res['t_prf'], res['prf_values'], marker='o', linestyle='-', color=color, label=f'prf ({label_prefix}): {res["median_prf"]:.1f}')
                if res['fit_params_pr'] is not None:
                    final_pressure_fit_r = res['fit_params_pr'][2]
                    self.ax_pr_final_pressure.axhline(y=final_pressure_fit_r, color=color, linestyle='--', label=f'Fit Final P ({label_prefix}): {final_pressure_fit_r:.2f}')

        self.ax_final_pressure.set_title('pbf Analysis')
        self.ax_final_pressure.set_xlabel('Time since Search Start (s)')
        self.ax_final_pressure.set_ylabel('pbf (ADC value)')
        self.ax_final_pressure.legend()
        self.ax_final_pressure.grid(True)

        self.ax_pr_final_pressure.set_title('prf Analysis')
        self.ax_pr_final_pressure.set_xlabel('Time since Search Start (s)')
        self.ax_pr_final_pressure.set_ylabel('prf (ADC value)')
        self.ax_pr_final_pressure.legend()
        self.ax_pr_final_pressure.grid(True)

        # --- Plot for Load vs Time in a separate window (Figure 2) ---
        self.time_fig.suptitle(f'Figure 2: Load & Geometry vs. Time for "{self.current_filename}"')
        if 'time' in self.df.columns:
            # --- Find the start of the static condition to define the plotting region ---
            if self.analysis_targets:
                plot_start_time = self.analysis_targets[0]['initial_search_time']
                plot_df = self.df[self.df['time'] >= plot_start_time].copy()
            else:
                plot_df = self.df.copy()

            stats_text = ""
            # --- "Time-Varying Pressure" line ---
            if not plot_df.empty:
                w_est_series = plot_df[col_w_calculated]
                avg_w_est = w_est_series.mean() # Overall average

                # --- Calculate Average Standard Deviation across segments ---
                segment_stds = []
                if self.analysis_targets:
                    for i, target_info in enumerate(self.analysis_targets):
                        start_time = target_info['initial_search_time']
                        # Define end time as start of next target, or end of data for the last target
                        end_time = self.analysis_targets[i+1]['initial_search_time'] if i + 1 < len(self.analysis_targets) else self.df['time'].iloc[-1]
                        
                        segment_mask = (plot_df['time'] >= start_time) & (plot_df['time'] < end_time)
                        segment_w_series = w_est_series[segment_mask]
                        if len(segment_w_series) > 1:
                            segment_stds.append(segment_w_series.std())

                if segment_stds:
                    avg_std_w_est = np.mean(segment_stds)
                else:
                    avg_std_w_est = w_est_series.std() # Fallback to overall std if no segments or targets

                stats_text = f'Time-Varying -> Avg: {avg_w_est:.1f} kg, Avg Std Dev: {avg_std_w_est:.1f} kg'
                self.ax_time.plot(plot_df['time'], w_est_series, label='Est. Mass (Time-Varying Pressure)')

            # --- Discontinuous "Predicted Settle Pressure" lines ---
            target_mass_values = []
            for i, res in enumerate(self.final_pressure_results):
                median_pbf = res.get('median_pbf')
                median_prf = res.get('median_prf')

                if median_pbf is None or median_prf is None:
                    continue

                start_time = res['initial_search_time']
                end_time = self.final_pressure_results[i+1]['initial_search_time'] if i + 1 < len(self.final_pressure_results) else self.df['time'].iloc[-1]

                plot_mask = (self.df['time'] >= start_time) & (self.df['time'] < end_time)
                segment_df = self.df[plot_mask]

                if segment_df.empty:
                    continue

                Fc_new = 2 * (Ab * median_pbf - Ar * median_prf) * toPa
                segment_theta_a = theta_a.loc[segment_df.index]

                if self.use_surf_fit:
                    segment_theta_g_raw = self.df[col_theta_g].loc[segment_df.index]
                    w_predicted, simple_lever_series = self.calculate_mass_surf_fit(segment_theta_a, Fc_new, segment_theta_g_raw)
                else:
                    w_predicted, simple_lever_series = self.calculate_mass(segment_theta_a, Fc_new)

                mass_at_start = w_predicted.iloc[0] if not w_predicted.empty else 0
                simple_lever_at_start = simple_lever_series.iloc[0] if not simple_lever_series.empty else 0
                target_mass_values.append(mass_at_start)

                # Get the actual theta_g (raw) at the start of the segment for the legend
                start_idx = (self.df['time'] - start_time).abs().idxmin()
                actual_theta_g_at_start = theta_g_raw.at[start_idx]
                
                # Store data for the export button
                self.surf_fit_export_data.append({'theta_g': actual_theta_g_at_start, 'mass': simple_lever_at_start})

                self.ax_time.plot(segment_df['time'], w_predicted,
                                  label=f'Est. Mass (θg {actual_theta_g_at_start:.2f}) ({mass_at_start:.1f} kg)',
                                  linestyle='--', color=res['color'])

            # --- Finalize Title with all stats ---
            target_stats_text = ""
            if len(target_mass_values) > 1:
                avg_target_mass = np.mean(target_mass_values)
                std_target_mass = np.std(target_mass_values)
                target_stats_text = f'Est. Mass (Tgt) -> Avg: {avg_target_mass:.1f} kg, Std: {std_target_mass:.1f} kg'

            title_lines = ['Estimated Load vs. Time (from static start)']
            if stats_text:
                title_lines.append(stats_text)
            if target_stats_text:
                title_lines.append(target_stats_text)
            self.ax_time.set_title('\n'.join(title_lines))

            self.ax_time.legend()
            self.ax_time.grid(True)
            self.ax_time.set_ylabel('Estimated Mass (w) [kg]')

            # --- Plot for Geometry Factor 'a' and theta_g ---
            if not plot_df.empty:
                theta_a_plot = theta_a.loc[plot_df.index]
                Lih = np.sqrt(Lgh**2 + Lgi**2 - 2 * Lgh * Lgi * np.cos(theta_a_plot + IGO)) # type: ignore
                GIH = np.arcsin(np.clip((Lgh / Lih) * np.sin(theta_a_plot + IGO), -1.0, 1.0))
                HIO = np.pi - IGO - GIH
                a_values = np.sin(HIO) - np.cos(HIO) * np.tan(theta_a_plot)

                # Plot 'a' on the primary y-axis
                p1 = self.ax_a.plot(plot_df['time'], a_values, label='Geometry Factor (a)', color='purple')
                self.ax_a.set_ylabel('Value of a', color='purple')
                self.ax_a.tick_params(axis='y', labelcolor='purple')
                self.ax_a.set_xlabel('Time (s)')
                self.ax_a.grid(True)

                # Use the existing secondary y-axis for theta_g
                p2 = self.ax_a2.plot(plot_df['time'], theta_a_plot, label='Boom Angle (θa)', color='green', linestyle=':')
                self.ax_a2.set_ylabel('θa (rad)', color='green')
                self.ax_a2.tick_params(axis='y', labelcolor='green')

                # Combine legends
                plots = p1 + p2
                labels = [p.get_label() for p in plots]
                self.ax_a.legend(plots, labels, loc='best')

        else:
            self.ax_time.text(0.5, 0.5, "'__time' column not found in CSV.",
                              horizontalalignment='center', verticalalignment='center',
                              transform=self.ax_time.transAxes)

        # --- Plot for Geometry Analysis (Figure 6) ---
        self.geometry_fig.suptitle(f'Figure 6: Geometry Analysis for "{self.current_filename}"\nIGO = {IGO:.2f} rad')
        
        time_axis = self.df['time']

        Lih_geom = np.sqrt(Lgh**2 + Lgi**2 - 2 * Lgh * Lgi * np.cos(theta_a + IGO))
        GIH_geom = np.arcsin(np.clip((Lgh / Lih_geom) * np.sin(theta_a + IGO), -1.0, 1.0))
        HIO_geom = np.pi - IGO - GIH_geom
        a_geom = np.sin(HIO_geom) - np.cos(HIO_geom) * np.tan(theta_a)

        # Plot Lih
        self.ax_lih.plot(time_axis, Lih_geom, label='Lih')
        self.ax_lih.set_title('Lih vs. Time')
        self.ax_lih.set_ylabel('Lih (m)')
        self.ax_lih.grid(True)

        # Plot GIH
        self.ax_gih.plot(time_axis, GIH_geom, label='GIH')
        self.ax_gih.set_title('GIH vs. Time')
        self.ax_gih.set_ylabel('GIH (rad)')
        self.ax_gih.grid(True)
        self.ax_gih.legend()

        # Plot HIO
        self.ax_hio.plot(time_axis, HIO_geom, label='HIO')
        self.ax_hio.set_title('HIO vs. Time')
        self.ax_hio.set_ylabel('HIO (rad)')
        self.ax_hio.grid(True)
        self.ax_hio.legend()

        # Plot a
        self.ax_a_geom.plot(time_axis, a_geom, label='a')
        self.ax_a_geom.set_title('a vs. Time')
        self.ax_a_geom.set_ylabel('a (factor)')
        self.ax_a_geom.grid(True)
        self.ax_a_geom.legend()

        # Plot theta_g
        self.ax_theta_g_geom.plot(time_axis, theta_a, label='θa')
        self.ax_theta_g_geom.set_title('θa vs. Time')
        self.ax_theta_g_geom.set_ylabel('θa (rad)')
        self.ax_theta_g_geom.set_xlabel('Time (s)')
        self.ax_theta_g_geom.grid(True)
        self.ax_theta_g_geom.legend()

        # --- Add vertical line and annotations for max GIH ---
        if not GIH_geom.empty:
            max_gih_idx = GIH_geom.idxmax()
            time_at_max_gih = time_axis.loc[max_gih_idx]
            
            # Values at max GIH
            max_gih_val = GIH_geom.loc[max_gih_idx]
            lih_at_max = Lih_geom.loc[max_gih_idx]
            hio_at_max = HIO_geom.loc[max_gih_idx]
            a_at_max = a_geom.loc[max_gih_idx]
            theta_a_at_max = theta_a.loc[max_gih_idx]

            # Draw vertical lines
            v_line1 = self.ax_lih.axvline(x=time_at_max_gih, color='r', linestyle='--', label=f'Max GIH at t={time_at_max_gih:.2f}s')
            v_line2 = self.ax_gih.axvline(x=time_at_max_gih, color='r', linestyle='--')
            v_line3 = self.ax_hio.axvline(x=time_at_max_gih, color='r', linestyle='--')
            v_line4 = self.ax_a_geom.axvline(x=time_at_max_gih, color='r', linestyle='--')
            v_line5 = self.ax_theta_g_geom.axvline(x=time_at_max_gih, color='r', linestyle='--')

            # Add text annotations
            text1 = self.ax_lih.text(time_at_max_gih, lih_at_max, f' {lih_at_max:.3f} m', va='bottom', ha='left', color='r', backgroundcolor='w')
            text2 = self.ax_gih.text(time_at_max_gih, max_gih_val, f' {max_gih_val:.3f} rad', va='bottom', ha='left', color='r', backgroundcolor='w')
            text3 = self.ax_hio.text(time_at_max_gih, hio_at_max, f' {hio_at_max:.3f} rad', va='bottom', ha='left', color='r', backgroundcolor='w')
            text4 = self.ax_a_geom.text(time_at_max_gih, a_at_max, f' {a_at_max:.3f}', va='bottom', ha='left', color='r', backgroundcolor='w')
            text5 = self.ax_theta_g_geom.text(time_at_max_gih, theta_a_at_max, f' {theta_a_at_max:.3f} rad', va='bottom', ha='left', color='r', backgroundcolor='w')

            # Store artists to toggle visibility
            self.max_gih_annotations.extend([v_line1, v_line2, v_line3, v_line4, v_line5, text1, text2, text3, text4, text5])

            # Set initial visibility based on checkbox
            is_visible = self.check_v_line_geom.get_status()[0]
            for artist in self.max_gih_annotations:
                artist.set_visible(is_visible)
        
        self.ax_lih.legend()

        self.kinematics_fig.canvas.draw_idle()
        self.time_fig.canvas.draw_idle()
        self.pressure_fig.canvas.draw_idle()
        self.final_pressure_fig.canvas.draw_idle()
        self.pr_pressure_fig.canvas.draw_idle()
        self.pr_final_pressure_fig.canvas.draw_idle()
        self.geometry_fig.canvas.draw_idle()

    def update_params(self, _):
        try:
            self.k1 = float(self.text_k1.text)
            self.k2 = float(self.text_k2.text)
        except ValueError:
            print(f"Invalid input for k1 or k2. Please enter valid numbers.")
            # Reset textboxes to last valid values
            self.text_k1.set_val(str(self.k1))
            self.text_k2.set_val(str(self.k2))
            return
        self.recalculate_and_plot()

    def toggle_compensation(self, label):
        # Get current statuses from widget after the click
        statuses = self.check_comp.get_status()
        comp_is_on, surf_is_on = statuses

        # If 'Surf Fit' was just turned on, turn 'Compensator 1' off.
        if label == 'Surf Fit' and surf_is_on:
            if comp_is_on:
                self.check_comp.set_active(0) # Toggle compensator off
        # If 'Compensator 1' was just turned on, turn 'Surf Fit' off.
        elif label == 'Compensator 1' and comp_is_on:
            if surf_is_on:
                self.check_comp.set_active(1) # Toggle surf fit off

        # Read the final, correct state from the widget after potential toggles
        self.use_compensation, self.use_surf_fit = self.check_comp.get_status()
        self.recalculate_and_plot()

    def toggle_max_gih_line(self, label):
        is_visible = self.check_v_line_geom.get_status()[0]
        for artist in self.max_gih_annotations:
            artist.set_visible(is_visible)
        
        # The legend on ax_lih depends on the visibility of the line.
        # We need to redraw the legend to show/hide the "Max GIH" entry.
        self.ax_lih.legend() 
        self.geometry_fig.canvas.draw_idle()


    def toggle_figure_visibility(self, label):
        """
        Shows or hides figures based on the check buttons.
        NOTE: This uses a backend-specific method (for TkAgg) to show/hide windows
        without closing them, which is cleaner than closing and recreating figures.
        It may not work with other matplotlib backends.
        """
        statuses = self.check_figs1.get_status() + self.check_figs2.get_status()
        kinematics_vis, time_vis, pressure_vis_pb, final_pressure_vis_pb, pressure_vis_pr, geom_vis, final_pressure_vis_pr, validation_vis = statuses

        def _toggle_win(fig, is_visible):
            """Helper to safely toggle a window's visibility."""
            manager = getattr(fig.canvas, 'manager', None)
            if manager and hasattr(manager, 'window') and manager.window:
                if is_visible:
                    manager.window.deiconify() # type: ignore
                else:
                    manager.window.withdraw() # type: ignore

        _toggle_win(self.kinematics_fig, kinematics_vis)
        _toggle_win(self.time_fig, time_vis)
        _toggle_win(self.pressure_fig, pressure_vis_pb)
        _toggle_win(self.final_pressure_fig, final_pressure_vis_pb)
        _toggle_win(self.pr_pressure_fig, pressure_vis_pr)
        _toggle_win(self.pr_final_pressure_fig, final_pressure_vis_pr)
        _toggle_win(self.geometry_fig, geom_vis)
        _toggle_win(self.validation_fig, validation_vis)

    def toggle_filters(self, label):
        self.use_median_filter, self.use_ema_filter, self.use_pressure_offset = self.check_filters.get_status()
        self.recalculate_and_plot()

    def update_filter_params(self, text):
        try:
            self.ema_alpha = float(self.text_ema_alpha.text)
            self.median_window = float(self.text_median_win.text)
        except ValueError:
            print(f"Invalid input for filter params. Please enter valid numbers.")
            self.text_ema_alpha.set_val(str(self.ema_alpha))
            self.text_median_win.set_val(str(self.median_window))
            return
        self.recalculate_and_plot()

    def toggle_theta_g_filter(self, label):
        self.use_theta_g_ema_filter = self.check_theta_g_filter.get_status()[0]
        self.recalculate_and_plot()

    def update_theta_g_filter_params(self, text):
        try:
            self.theta_g_ema_alpha = float(self.text_theta_g_ema_alpha.text)
        except ValueError:
            print(f"Invalid input for theta_g EMA alpha. Please enter a valid number.")
            self.text_theta_g_ema_alpha.set_val(str(self.theta_g_ema_alpha))
            return
        self.recalculate_and_plot()

    def update_sensor_scale_factor(self, text):
        try:
            self.sensor_scale_factor = float(text)
        except ValueError:
            print(f"Invalid input for Sensor Scale Factor: '{text}'. Please enter a valid number.")
            return
        self.recalculate_and_plot()

    def update_theta_g_offset(self, text):
        try:
            self.theta_g_offset = float(text)
        except ValueError:
            print(f"Invalid input for theta_g offset: '{text}'. Please enter a valid number.")
            self.text_theta_g_offset.set_val(str(self.theta_g_offset))
            return
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

    def update_slope_params(self, text):
        try:
            self.slope_time_interval = float(self.text_slope_interval.text)
        except ValueError:
            print(f"Invalid input for Slope Interval: '{text}'. Please enter a valid number.")
            self.text_slope_interval.set_val(str(self.slope_time_interval))
            return
        self.recalculate_and_plot()

    def update_settling_params(self, text_or_val):
        # This handler is called by both the slider and the textbox
        try:
            self.slope_threshold = float(self.text_slope_thresh.text)
            self.settling_duration_req = float(self.text_settling_dur_req.text)
        except ValueError:
            print(f"Invalid input for Stable Duration or Slope Threshold. Please enter valid numbers.")
            self.text_settling_dur_req.set_val(str(self.settling_duration_req))
            self.text_slope_thresh.set_val(str(self.slope_threshold))
            return
        self.recalculate_and_plot()

    def update_auto_time_params(self, text):
        try:
            targets_str = self.text_target_theta.text.split(',')
            self.target_theta_g_list = [float(t.strip()) for t in targets_str if t.strip()]
            self.theta_g_threshold = float(self.text_theta_thresh.text)
            self.vel_g_threshold_auto = float(self.text_vel_thresh.text)
            self.acc_g_threshold_auto = float(self.text_acc_thresh.text)
            print(f"Updated auto-detect targets to: {self.target_theta_g_list}")
        except ValueError:
            print(f"Invalid input: '{text}'. Please enter valid, comma-separated numbers.")
            return
        self.run_auto_initial_time_detection()

    def toggle_slope_filter(self, label):
        self.use_slope_filter = self.check_slope_filter.get_status()[0]
        self.recalculate_and_plot()

    def toggle_fill_targets(self, label):
        self.fill_common_targets = self.check_fill_targets.get_status()[0]
        if self.fill_common_targets:
            common_targets = "0.7,0.6,0.5,0.4,0.3,0.2,0.1"
            self.text_target_theta.set_val(common_targets)
            # Manually trigger the update
            self.update_auto_time_params(common_targets)

    def toggle_auto_theta(self, label):
        self.auto_set_theta_g = self.check_auto_theta.get_status()[0]
        if self.auto_set_theta_g:
            # When enabling, re-parse filename and update textbox and plots
            match = re.search(r'(\d+\.\d+)', self.current_filename)
            default_target = 0.3
            if match:
                try:
                    self.target_theta_g_list = [float(match.group(1))]
                    print(f"Auto-set target_theta_g from filename to: {self.target_theta_g_list}")
                except (ValueError, IndexError):
                    self.target_theta_g_list = [default_target]
                    print(f"Could not parse float from filename. Defaulting target_theta_g to: {[default_target]}")
            else:
                self.target_theta_g_list = [default_target]
                print(f"Filename does not contain a float. Defaulting target_theta_g to: {[default_target]}")

            self.text_target_theta.set_val(', '.join(map(str, self.target_theta_g_list)))
            self.run_auto_initial_time_detection()

    def run_auto_initial_time_detection(self):
        self.analysis_targets = []
        for target_g in self.target_theta_g_list:
            # Find the time when theta_g is closest to the target value.
            closest_idx = (self.df[col_theta_g] - target_g).abs().idxmin()
            t_target_center = self.df.at[closest_idx, 'time']

            # Define the search window of [-200, +200] seconds
            search_start = max(0, t_target_center - 200)
            search_end = t_target_center + 200

            # Create masks for the search conditions
            time_window_mask = (self.df['time'] >= search_start) & (self.df['time'] <= search_end)
            cond_theta = (self.df[col_theta_g] >= target_g - self.theta_g_threshold) & \
                         (self.df[col_theta_g] <= target_g + self.theta_g_threshold)
            cond_vel = self.df[col_vel_g].abs() < self.vel_g_threshold_auto
            cond_acc = self.df[col_acc_g].abs() < self.acc_g_threshold_auto

            # Find all matching points within the window
            match_df = self.df[time_window_mask & cond_theta & cond_vel & cond_acc]

            if not match_df.empty:
                auto_initial_time = match_df['time'].iloc[0]
                print(f"Auto-detect: Found static point for target {target_g:.2f} at time {auto_initial_time:.2f}s (searched [{search_start:.2f}s, {search_end:.2f}s])")
                self.analysis_targets.append({'target_g': target_g, 'initial_search_time': auto_initial_time})
            else:
                print(f"Auto-detect: No static point found for target {target_g:.2f} in window [{search_start:.2f}s, {search_end:.2f}s]")

        # Sort targets by their search time to process them in chronological order
        self.analysis_targets.sort(key=lambda x: x['initial_search_time'])

        # Manually trigger a replot with the new initial time.
        self.recalculate_and_plot()

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
        if self.auto_set_theta_g:
            match = re.search(r'(\d+\.\d+)', self.current_filename)
            default_target = 0.3
            if match:
                try:
                    self.target_theta_g_list = [float(match.group(1))]
                    print(f"Auto-set target_theta_g from filename to: {self.target_theta_g_list}")
                except (ValueError, IndexError):
                    self.target_theta_g_list = [default_target]
                    print(f"Could not parse float from filename. Defaulting target_theta_g to: {[default_target]}")
            else:
                self.target_theta_g_list = [default_target]
                print(f"Filename does not contain a float. Defaulting target_theta_g to: {[default_target]}")

            self.text_target_theta.set_val(', '.join(map(str, self.target_theta_g_list)))

        self.run_auto_initial_time_detection()

    def open_file_dialog(self, event):
        """
        Opens a file dialog to select one or more CSV files.
        If multiple files are selected, it processes each one and automatically
        exports the results to 'surffit.csv'.
        """
        # We don't need to create and destroy a new Tk root window here.
        # Matplotlib's TkAgg backend already manages a root window, and creating
        # a new one and destroying it can interfere with the main event loop.
        filepaths = filedialog.askopenfilenames(
            initialdir=os.path.expanduser('~/wheel_loader_ws/results/csv/settle/'),
            title="Select one or more CSV files",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*"))
        )

        if filepaths:
            print(f"\n--- Processing {len(filepaths)} files for surf fit export ---")
            for filepath in filepaths:
                self.load_file(filepath)
                self.export_for_surf_fit(None) # Auto-export for each file
            print("--- All files processed and exported. ---")
        else:
            print("File selection cancelled.")

    def export_for_surf_fit(self, event):
        """
        Exports the calculated mass for each target theta_g into a wide-format CSV
        named 'surffit.csv', suitable for surface fitting analysis.
        """
        output_filename = os.path.join(self.workspace_root, 'surffit.csv')
        
        # 1. Check if there is data to export
        if not self.surf_fit_export_data:
            print("Export Error: No target mass data available to export. Please run an analysis first.")
            return

        # 2. Parse the load value from the current filename to use as a column header
        # Example: "downstair_3.15" -> "3.15t"
        match = re.search(r'(\d+\.?\d*)', self.current_filename)
        if not match:
            print(f"Export Error: Could not parse load value (e.g., '3.15') from filename '{self.current_filename}' to create CSV header.")
            return
        load_val_str = match.group(1)
        new_col_name = f"{load_val_str}t"

        # 3. Prepare the new data into a DataFrame, rounding theta_g and ensuring uniqueness
        new_data_list = [{'theta_g': d['theta_g'], new_col_name: d['mass']} for d in self.surf_fit_export_data]
        new_df = pd.DataFrame(new_data_list)
        new_df['theta_g'] = new_df['theta_g'].round(1)
        # If multiple static points round to the same theta_g, take their average mass
        new_df = new_df.groupby('theta_g', as_index=False).mean()

        # 4. Read existing file or create a new DataFrame if it doesn't exist
        if os.path.exists(output_filename):
            try:
                existing_df = pd.read_csv(output_filename)
                if new_col_name in existing_df.columns:
                    existing_df = existing_df.drop(columns=[new_col_name])
                
                final_df = pd.merge(existing_df, new_df, on='theta_g', how='outer')
            except Exception as e:
                print(f"Error reading or merging with existing '{output_filename}': {e}")
                print("Creating a new file instead.")
                final_df = new_df
        else:
            final_df = new_df

        # 5. Consolidate the final dataframe to ensure one row per theta_g
        # This cleans up duplicates from previous buggy exports and the current merge.
        # We group by 'theta_g' and take the first non-null value for each load column.
        final_df = final_df.groupby('theta_g', as_index=False).first()

        # 6. Sort by theta_g (descending) and save to CSV
        try:
            final_df = final_df.sort_values(by='theta_g', ascending=False).reset_index(drop=True)
            final_df.to_csv(output_filename, index=False, float_format='%.1f')
            print(f"Successfully exported/updated data to '{os.path.abspath(output_filename)}'")
        except Exception as e:
            print(f"Error saving data to '{output_filename}': {e}")

    def validate_and_export(self, event):
        """
        Reads the INPUT data (w and theta_g) from surffit.csv, applies the CURRENT
        estimation model to get the final load, calculates statistics, and saves
        the results to validate.csv.
        """
        source_filename = os.path.join(self.workspace_root, 'surffit.csv')
        output_filename = os.path.join(self.workspace_root, 'validate.csv')

        if not os.path.exists(source_filename):
            print(f"Error: Source file '{source_filename}' not found. Please use 'Export for Surf Fit' first.")
            return

        try:
            df_input = pd.read_csv(source_filename)
        except Exception as e:
            print(f"Error reading '{source_filename}': {e}")
            return

        # Create a new dataframe to store the results of the CURRENT model
        df_results = df_input.copy()

        # Identify load columns and theta_g column
        if 'theta_g' not in df_results.columns:
            print("Error: 'theta_g' column not found in surffit.csv.")
            return
        
        load_cols = sorted([col for col in df_results.columns if col.endswith('t') and col[0].isdigit()], key=lambda x: float(x.replace('t','')))
        if not load_cols:
            print("Error: No load columns (e.g., '1.2t') found in surffit.csv.")
            return

        # --- Re-calculate final estimated load (yy) using the CURRENT model settings ---
        for col in load_cols:
            for index, row in df_results.iterrows():
                w = row[col] # This is simple_lever from surffit.csv
                theta_g_raw = row['theta_g']
                
                if pd.isna(w) or pd.isna(theta_g_raw):
                    continue

                # Apply the currently selected compensation model
                if self.use_surf_fit:
                    scope = {'w': w, 'theta_g': theta_g_raw, 'np': np}
                    final_mass_tons = eval(self.surf_fit_rhs, {'np': np}, scope)
                    final_mass_kg = final_mass_tons * 1000
                elif self.use_compensation:
                    theta_a = theta_g_raw + self.theta_g_offset
                    compensator = self.k1 * theta_a + self.k2
                    final_mass_kg = w + compensator
                else: # No compensation
                    final_mass_kg = w
                
                df_results.at[index, col] = final_mass_kg

        # --- Now perform validation on the newly calculated df_results ---
        # --- 1. Calculate RMSE_s (row-wise) ---
        rmse_s_values = []
        actual_loads_kg = np.array([float(col.replace('t', '')) * 1000 for col in load_cols])

        for index, row in df_results.iterrows():
            predicted_loads = row[load_cols].to_numpy(dtype=float)
            
            valid_mask = ~np.isnan(predicted_loads)
            if not np.any(valid_mask):
                rmse_s_values.append(np.nan)
                continue

            valid_predicted = predicted_loads[valid_mask]
            valid_actual = actual_loads_kg[valid_mask]

            squared_errors = (valid_predicted - valid_actual) ** 2
            rmse_s = np.sqrt(np.mean(squared_errors))
            rmse_s_values.append(rmse_s)
        
        df_results['RMSE_s'] = rmse_s_values

        # --- 2. Calculate avg and RMSE_w (column-wise) ---
        avg_row = {'theta_g': 'avg'}
        rmse_w_row = {'theta_g': 'RMSE_w'}

        for col in load_cols:
            predicted_col = df_results[col].dropna()
            
            if not predicted_col.empty:
                avg_row[col] = predicted_col.mean()
                actual_load_kg = float(col.replace('t', '')) * 1000
                squared_errors_col = (predicted_col - actual_load_kg) ** 2
                rmse_w = np.sqrt(squared_errors_col.mean())
                rmse_w_row[col] = rmse_w
            else:
                avg_row[col] = np.nan
                rmse_w_row[col] = np.nan
        
        summary_df = pd.DataFrame([avg_row, rmse_w_row])

        # --- 3. Combine and save ---
        df_results['theta_g'] = df_results['theta_g'].astype(object)
        validate_df = pd.concat([df_results, summary_df], ignore_index=True)

        try:
            validate_df.to_csv(output_filename, index=False, float_format='%.1f')
            print(f"Successfully created validation file with CURRENT model data: '{os.path.abspath(output_filename)}'")
        except Exception as e:
            print(f"Error saving data to '{output_filename}': {e}")
            return

        # --- 4. Visualize the validation results ---
        try:
            # Check if the figure has been closed by the user. If so, recreate it.
            # Using plt.fignum_exists is a backend-agnostic way to check.
            if not plt.fignum_exists(self.validation_fig.number):
                print("Validation figure was closed. Recreating it.")
                self.validation_fig, (self.ax_rmse_s, self.ax_rmse_w) = plt.subplots(2, 1, num='Figure 8: Validation', figsize=(10, 8))
                self.validation_fig.subplots_adjust(left=0.08, bottom=0.08, right=0.95, top=0.92, hspace=0.4)
                
                # Ensure the checkbox in the control panel reflects that the figure is now visible.
                # 'Validation' is the 4th item in the 2nd list (index 3).
                if not self.check_figs2.get_status()[3]:
                    self.check_figs2.set_active(3)

            self.ax_rmse_s.clear()
            self.ax_rmse_w.clear()

            # Determine which equation is being used for the title
            if self.use_surf_fit:
                equation_str = f"Using Surface Fit: load = {self.surf_fit_rhs}"
            elif self.use_compensation:
                equation_str = f"Using Compensator: w_comp = {self.k1:.2f}*θa + {self.k2:.2f}"
            else:
                equation_str = "Using Simple Lever Model (No Compensation)"

            # Subplot 1: RMSE_s vs theta_g
            plot_df = validate_df[pd.to_numeric(validate_df['theta_g'], errors='coerce').notna()].copy()
            if not plot_df.empty:
                plot_df['theta_g'] = plot_df['theta_g'].astype(float)
                plot_df = plot_df.sort_values(by='theta_g')
                
                self.ax_rmse_s.bar(plot_df['theta_g'].astype(str), plot_df['RMSE_s'], color='skyblue', edgecolor='black')
                self.ax_rmse_s.set_title('Model Accuracy at different Boom Angles (RMSE_s)')
                self.ax_rmse_s.set_xlabel('Boom Angle, theta_g (rad)')
                self.ax_rmse_s.set_ylabel('RMSE_s (kg)')
                self.ax_rmse_s.grid(axis='y', linestyle='--', alpha=0.7)

            # Subplot 2: RMSE_w vs Load
            if 'RMSE_w' in validate_df['theta_g'].values:
                rmse_w_series = validate_df[validate_df['theta_g'] == 'RMSE_w'].iloc[0]
                rmse_w_values = rmse_w_series[load_cols].astype(float)

                self.ax_rmse_w.bar(load_cols, rmse_w_values, color='lightgreen', edgecolor='black')
                self.ax_rmse_w.set_title('Model Accuracy across different Loads (RMSE_w)')
                self.ax_rmse_w.set_xlabel('Actual Load (t)')
                self.ax_rmse_w.set_ylabel('RMSE_w (kg)')
                self.ax_rmse_w.grid(axis='y', linestyle='--', alpha=0.7)

            self.validation_fig.suptitle(f'Validation Analysis\n({equation_str})', fontsize=12)
            self.validation_fig.tight_layout(rect=[0, 0.03, 1, 0.92])
            self.validation_fig.canvas.draw_idle()

            # Bring the validation figure to the front
            manager = self.validation_fig.canvas.manager
            # Use winfo_exists() for a robust check with the Tk backend to prevent crash
            if manager and hasattr(manager, 'window') and manager.window.winfo_exists():
                manager.window.deiconify()
                manager.window.lift()
                manager.window.attributes('-topmost', 1)
                manager.window.attributes('-topmost', 0)
        except Exception as e:
            print(f"Error during validation plotting: {e}")

    def set_and_apply_surf_fit_equation(self, eq_rhs):
        """Updates the surface fit equation and replots."""
        print(f"Applying new surface fit equation: {eq_rhs}")
        self.surf_fit_rhs = eq_rhs
        self.recalculate_and_plot()

    def apply_deg1_equation(self, event):
        """Applies the stored Degree 1 equation."""
        self.set_and_apply_surf_fit_equation(self.eq_deg1_rhs)

    def apply_best_equation(self, event):
        """Applies the stored Best Fit equation."""
        self.set_and_apply_surf_fit_equation(self.eq_best_rhs)

    def process_ui_updates(self):
        """
        Checks the queue for UI updates from background threads and processes them.
        This runs on the main GUI thread.
        """
        try:
            # Get an update from the queue without blocking
            status, data = self.ui_update_queue.get_nowait()
            self._update_ui_with_fit_results(status, data)
        except queue.Empty:
            pass  # No updates in the queue
        finally:
            # Reschedule this method to run again after 100ms
            self.control_fig.canvas.get_tk_widget().after(100, self.process_ui_updates)

    def _update_ui_with_fit_results(self, status, data=None):
        """
        Safely updates the GUI with results from the background thread.
        This method is designed to be called via `canvas.after()`.
        """
        if status == "error_script_not_found":
            self.text_eq_deg1.set_text("Eq Deg 1: Script not found.")
            self.text_eq_best.set_text("Eq Best: Script not found.")
        elif status == "error_running_script":
            self.text_eq_deg1.set_text("Eq Deg 1: Error running script.")
            self.text_eq_best.set_text("Eq Best: Error running script.")
        elif status == "error_parsing":
            self.text_eq_deg1.set_text("Eq Deg 1: Error parsing script output.")
            self.text_eq_best.set_text("Eq Best: Error parsing script output.")
        elif status == "success" and data:
            eq_deg1_full, self.eq_deg1_rhs, eq_best_full, self.eq_best_rhs, self.eq_best_degree = data
            self.text_eq_deg1.set_text(eq_deg1_full)
            self.text_eq_best.set_text(eq_best_full)
            self.btn_apply_deg1.ax.set_visible(True)
            self.btn_apply_best.ax.set_visible(self.eq_best_degree > 1)
        
        self.control_fig.canvas.draw_idle()

    def find_surf_fit_equations(self, event):
        """
        Runs surface_fit.py in a background thread to avoid freezing the GUI,
        and then updates the UI with the found equations.
        """
        def worker():
            """The function that will run in the background thread."""
            script_path = os.path.join(os.path.dirname(__file__), 'surface_fit.py')
            if not os.path.exists(script_path):
                print(f"Error: Script not found: {script_path}")
                self.ui_update_queue.put(("error_script_not_found", None))
                return

            try:
                # Execute the script and capture output
                result = subprocess.run(
                    ['python3', script_path, '--no-plot'],
                    capture_output=True,
                    text=True,
                    check=True,
                    cwd=os.path.expanduser('~/wheel_loader_ws/') # Run from ws root to find surffit.csv
                )
                output = result.stdout
                print(output)
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                error_msg = f"Failed to execute surface_fit.py. Error: {e}"
                print(error_msg)
                if hasattr(e, 'stderr'):
                    print(f"\nStderr:\n{e.stderr}")
                self.ui_update_queue.put(("error_running_script", None))
                return

            try:
                deg1_match = re.search(r"--- Degree 1 Equation ---\s*\n(.*?)\n", output, re.DOTALL)
                best_fit_match = re.search(r"--- Best Fit \(Degree (\d+)\) Equation ---\s*\n(.*?)\n", output, re.DOTALL)
                if not deg1_match:
                    raise ValueError("Could not parse Degree 1 equation from script output.")

                eq_deg1_full = deg1_match.group(1).strip()
                eq_deg1_rhs = eq_deg1_full.split(' = ', 1)[1]

                best_degree = 1
                if best_fit_match:
                    best_degree = int(best_fit_match.group(1))
                    eq_best_full = best_fit_match.group(2).strip()
                    eq_best_rhs = eq_best_full.split(' = ', 1)[1]
                else:
                    eq_best_rhs = eq_deg1_rhs
                    eq_best_full = eq_deg1_full
                
                results_data = (eq_deg1_full, eq_deg1_rhs, eq_best_full, eq_best_rhs, best_degree)
                self.ui_update_queue.put(("success", results_data))

            except (ValueError, IndexError) as e:
                print(f"Could not parse equations from surface_fit.py output.\nError: {e}")
                self.ui_update_queue.put(("error_parsing", None))

        # This part runs in the main GUI thread
        print("Running surface_fit.py to find new equations...")
        self.text_eq_deg1.set_text("Eq Deg 1: Running...")
        self.text_eq_best.set_text("Eq Best: Running...")
        self.control_fig.canvas.draw_idle()
        
        # Create and start the background thread
        thread = threading.Thread(target=worker)
        thread.daemon = True  # Allows main program to exit even if the thread is running
        thread.start()


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