#!/usr/bin/python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.widgets import Slider, CheckButtons, RadioButtons, Button
import tkinter as tk
from tkinter import filedialog

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
        self.use_Ar = True
        self.Ar = 0.014  # m^2, piston cross-section area
        self.use_median_filter = True
        self.use_ema_filter = False
        self.ema_alpha = 0.1
        self.median_window = 101
        self.slope_time_interval = 1.0

        # --- Create Figure and Axes for plots---
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
        self.fig.subplots_adjust(hspace=0.3)

        # --- Create a new Figure for the time plot ---
        self.time_fig, self.ax_time = plt.subplots(1, 1, num='Figure 2: Load vs Time', figsize=(12, 6))

        # --- Create a new Figure for pressure analysis ---
        self.pressure_fig, ((self.ax_pressure_pb, self.ax_pressure_pr), (self.ax_slope_pb, self.ax_slope_pr)) = plt.subplots(2, 2, num='Figure 3: Pressure Analysis', figsize=(12, 9), sharex='col')
        self.pressure_fig.subplots_adjust(hspace=0.4, wspace=0.3)

        # --- Create a separate Figure for controls ---
        self.control_fig = plt.figure("Tuning Controls", figsize=(7, 7))
        self.control_fig.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)

        # --- Create UI Controls in the new window ---
        # Bottom: File Selection Button
        ax_select_btn = self.control_fig.add_axes([0.1, 0.05, 0.8, 0.075])
        # Middle: Filter Controls
        ax_filter_checks = self.control_fig.add_axes([0.05, 0.38, 0.15, 0.15])
        ax_ema_alpha = self.control_fig.add_axes([0.25, 0.45, 0.65, 0.05])
        ax_median_win = self.control_fig.add_axes([0.25, 0.38, 0.65, 0.05])
        ax_slope_interval = self.control_fig.add_axes([0.25, 0.31, 0.65, 0.05])
        # Top: Compensation Controls (re-arranged for clarity)
        y_pos = 0.9
        ax_k1 = self.control_fig.add_axes([0.25, y_pos, 0.65, 0.05])
        ax_check_link = self.control_fig.add_axes([0.05, y_pos, 0.15, 0.05])
        y_pos -= 0.1
        ax_k2 = self.control_fig.add_axes([0.25, y_pos, 0.65, 0.05])
        y_pos -= 0.1
        ax_k3 = self.control_fig.add_axes([0.25, y_pos, 0.65, 0.05])
        ax_check_bucket = self.control_fig.add_axes([0.05, y_pos, 0.15, 0.05])
        y_pos -= 0.1
        ax_k4 = self.control_fig.add_axes([0.25, y_pos, 0.65, 0.05])
        ax_check_ar = self.control_fig.add_axes([0.05, y_pos, 0.15, 0.05])

        self.slider_k1 = Slider(ax_k1, 'k1', 0, 5000, valinit=self.k1)
        self.slider_k2 = Slider(ax_k2, 'k2', 0.0, 1.0, valinit=self.k2)
        self.slider_k3 = Slider(ax_k3, 'k3', 0, 5000, valinit=self.k3)
        self.slider_k4 = Slider(ax_k4, 'k4', -1.0, 1.0, valinit=self.k4)
        self.check_comp = CheckButtons(ax_check_link, ['Link Comp.'], [self.use_compensation])
        self.check_bucket_comp = CheckButtons(ax_check_bucket, ['Bucket Comp.'], [self.use_bucket_compensation])
        self.check_ar = CheckButtons(ax_check_ar, ['Use Ar'], [self.use_Ar])
        self.btn_select_file = Button(ax_select_btn, 'Select CSV File')

        self.check_filters = CheckButtons(ax_filter_checks, ['Median', 'EMA'], actives=[self.use_median_filter, self.use_ema_filter])
        self.slider_ema_alpha = Slider(ax_ema_alpha, 'EMA Alpha', 0.01, 1.0, valinit=self.ema_alpha)
        self.slider_median_win = Slider(ax_median_win, 'Median Win', 3, 101, valinit=self.median_window, valstep=2)
        self.slider_slope_interval = Slider(ax_slope_interval, 'Slope Interval (s)', 0.1, 5.0, valinit=self.slope_time_interval, valstep=0.1)

        # --- Connect UI Events ---
        self.btn_select_file.on_clicked(self.open_file_dialog)
        self.slider_k1.on_changed(self.update_params)
        self.slider_k2.on_changed(self.update_params)
        self.slider_k3.on_changed(self.update_params)
        self.slider_k4.on_changed(self.update_params)
        self.check_comp.on_clicked(self.toggle_compensation)
        self.check_bucket_comp.on_clicked(self.toggle_bucket_compensation)
        self.check_ar.on_clicked(self.toggle_Ar)
        self.check_filters.on_clicked(self.toggle_filters)
        self.slider_ema_alpha.on_changed(self.update_filter_params)
        self.slider_median_win.on_changed(self.update_filter_params)
        self.slider_slope_interval.on_changed(self.update_slope_params)

        # --- Initial Calculation and Plot ---
        self.recalculate_and_plot()

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
        slope_pr = pr_filtered.diff(periods=samples_per_interval) / time_delta

        static_df = self.df[
            (self.df[col_vel_g].abs() < VEL_THRESHOLD) &
            (self.df[col_acc_g].abs() < ACC_THRESHOLD)
        ].copy()

        # --- Clear and Redraw Plots ---
        self.ax1.clear()
        self.ax2.clear()
        self.ax_time.clear()
        self.ax_pressure_pb.clear()
        self.ax_pressure_pr.clear()
        self.ax_slope_pb.clear()
        self.ax_slope_pr.clear()

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
            self.ax_pressure_pb.set_title('Raw vs. Filtered Pressure (pb)')
            self.ax_pressure_pb.set_ylabel('Pressure (ADC value)')
            self.ax_pressure_pb.legend(fontsize='small')
            self.ax_pressure_pb.grid(True)

            # --- Top-right: Raw vs Filtered Pressure (pr) ---
            self.ax_pressure_pr.plot(self.df['time'], self.df[col_pr], label='Raw pr', color='magenta', alpha=0.7)
            self.ax_pressure_pr.plot(self.df['time'], pr_filtered, label=f'Filtered pr ({self.applied_filters_str})', color='red')
            self.ax_pressure_pr.set_title('Raw vs. Filtered Pressure (pr)')
            self.ax_pressure_pr.legend(fontsize='small')
            self.ax_pressure_pr.grid(True)

            # --- Bottom-left: Slope of pb ---
            self.ax_slope_pb.plot(self.df['time'], slope_pb, label='Slope pb', color='blue')
            self.ax_slope_pb.set_title('Slope of Filtered pb')
            self.ax_slope_pb.set_ylabel('Pressure Slope')
            self.ax_slope_pb.set_xlabel('Time (s)')
            self.ax_slope_pb.grid(True)

            # --- Bottom-right: Slope of pr ---
            self.ax_slope_pr.plot(self.df['time'], slope_pr, label='Slope pr', color='red')
            self.ax_slope_pr.set_title('Slope of Filtered pr')
            self.ax_slope_pr.set_xlabel('Time (s)')
            self.ax_slope_pr.grid(True)

        self.time_fig.canvas.draw_idle()
        self.pressure_fig.canvas.draw_idle()
        self.fig.canvas.draw_idle()

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

    def toggle_Ar(self, label):
        self.use_Ar = self.check_ar.get_status()[0]
        if self.use_Ar:
            self.Ar = 0.014
        else:
            self.Ar = 0.0
        self.recalculate_and_plot()

    def toggle_filters(self, label):
        self.use_median_filter, self.use_ema_filter = self.check_filters.get_status()
        self.recalculate_and_plot()

    def update_filter_params(self, val):
        self.ema_alpha = self.slider_ema_alpha.val
        self.median_window = self.slider_median_win.val
        self.recalculate_and_plot()

    def update_slope_params(self, val):
        self.slope_time_interval = self.slider_slope_interval.val
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
        self.recalculate_and_plot()

    def open_file_dialog(self, event):
        """Opens a file dialog to select a CSV file."""
        root = tk.Tk()
        root.withdraw() # Hide the main tkinter window
        root.attributes('-topmost', True) # Bring the dialog to the front
        filepath = filedialog.askopenfilename(
            initialdir=os.path.expanduser('~/wheel_loader_ws/results/csv/'),
            title="Select a CSV file",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*"))
        )
        root.destroy()

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
    initial_filename = 'test5_empty_final'
    csv_file_path = os.path.expanduser(f'~/wheel_loader_ws/results/csv/{initial_filename}.csv')

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
