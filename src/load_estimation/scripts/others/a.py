#!/usr/bin/python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.widgets import Slider, CheckButtons, RadioButtons

# --- Constants ---
Lgh = 0.163
Lgi = np.sqrt(0.135**2 + 0.64**2)
Lag = 3.9294
IGO = np.pi - np.arctan2(0.64, 0.135)

# --- Thresholds for Static Condition ---
ACC_THRESHOLD = 0.001  # rad/s^2
VEL_THRESHOLD = 0.003  # rad/s

# --- Configuration ---
FILENAMES = [
    'test5_empty_final', 
    'test5_twohuman2_final', 
    'test5_threebricks_final', 
    'test5_sixbricks2_final', 
    'test5_ninebricks_final', 
    'test5_tenbricks_singlehuman2_final',
    'load1.5T_up',
    'load2.5T_up']

ACTUAL_LOAD = [0, 150, 105, 210, 315, 425, 1500, 2500]  

# Column names from the CSV file
col_theta_g = '/debug/theta_g'
col_f = '/debug/f'
col_acc_g = 'acc_g'
col_vel_g = 'vel_g'
col_w_calculated = 'w_calculated'
col_w_csv = '/debug/w'


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

        # --- Create Figure and Axes for plots---
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
        self.fig.subplots_adjust(hspace=0.3)

        # --- Create a new Figure for the time plot ---
        self.time_fig, self.ax_time = plt.subplots(1, 1, num='Load vs Time', figsize=(12, 6))

        # --- Create a separate Figure for controls ---
        self.control_fig = plt.figure("Tuning Controls", figsize=(7, 5))
        self.control_fig.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.1)

        # --- Create UI Controls in the new window ---
        ax_select = self.control_fig.add_axes([0.1, 0.05, 0.8, 0.35])
        ax_k1 = self.control_fig.add_axes([0.25, 0.8, 0.65, 0.1])
        ax_k2 = self.control_fig.add_axes([0.25, 0.65, 0.65, 0.1])
        ax_k3 = self.control_fig.add_axes([0.25, 0.5, 0.65, 0.1])
        ax_k4 = self.control_fig.add_axes([0.25, 0.35, 0.65, 0.1])
        ax_check_link = self.control_fig.add_axes([0.05, 0.8, 0.15, 0.1])
        ax_check_bucket = self.control_fig.add_axes([0.05, 0.65, 0.15, 0.1])

        self.slider_k1 = Slider(ax_k1, 'k1', 0, 5000, valinit=self.k1)
        self.slider_k2 = Slider(ax_k2, 'k2', 0.0, 1.0, valinit=self.k2)
        self.slider_k3 = Slider(ax_k3, 'k3', 0, 5000, valinit=self.k3)
        self.slider_k4 = Slider(ax_k4, 'k4', -1.0, 1.0, valinit=self.k4)
        self.check_comp = CheckButtons(ax_check_link, ['Link Comp.'], [self.use_compensation])
        self.check_bucket_comp = CheckButtons(ax_check_bucket, ['Bucket Comp.'], [self.use_bucket_compensation])
        self.radio_select = RadioButtons(ax_select, FILENAMES, active=FILENAMES.index(self.current_filename))

        # --- Connect UI Events ---
        self.radio_select.on_clicked(self.select_file)
        self.slider_k1.on_changed(self.update_params)
        self.slider_k2.on_changed(self.update_params)
        self.slider_k3.on_changed(self.update_params)
        self.slider_k4.on_changed(self.update_params)
        self.check_comp.on_clicked(self.toggle_compensation)
        self.check_bucket_comp.on_clicked(self.toggle_bucket_compensation)

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
        self.df[col_w_calculated] = self.calculate_mass(theta_g, self.df[col_f])

        # --- (Subplot 3 code is commented out, so its data calculation is also commented) ---
        # if self.use_compensation:
        #     link_compensate_plot = self.k1 * np.cos(theta_g + self.k2) / np.cos(theta_g)
        # else:
        #     link_compensate_plot = np.zeros_like(theta_g)

        static_df = self.df[
            (self.df[col_vel_g].abs() < VEL_THRESHOLD) &
            (self.df[col_acc_g].abs() < ACC_THRESHOLD)
        ].copy()

        # --- Clear and Redraw Plots ---
        self.ax1.clear()
        self.ax2.clear()
        self.ax_time.clear()

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

            # --- Stats for w from the original CSV ---
            w_csv_series = static_df[col_w_csv]
            avg_w_csv = w_csv_series.mean()
            std_w_csv = w_csv_series.std()

            stats_text = (f'Live Calc -> Avg: {avg_w_calc:.1f}, Std: {std_w_calc:.1f}\n'
                          f'CSV Data -> Avg: {avg_w_csv:.1f}, Std: {std_w_csv:.1f}')

            # Plot w from CSV
            self.ax2.scatter(static_df[col_theta_g], w_csv_series, alpha=0.5, s=15,
                             color='blue', marker='x', label='w (from CSV)')
            # Plot w calculated by this script
            self.ax2.scatter(static_df[col_theta_g], w_calc_series, alpha=0.5, s=15,
                             color='green', marker='o', label='w (Live Calc)')

            # Add lines for actual load and averages
            file_index = FILENAMES.index(self.current_filename)
            actual_load = ACTUAL_LOAD[file_index]
            self.ax2.axhline(y=actual_load, color='red', linestyle='--', linewidth=2, label=f'Actual: {actual_load} kg')
            self.ax2.axhline(y=avg_w_calc, color='green', linestyle='--', linewidth=2, label=f'Live Calc Avg: {avg_w_calc:.1f} kg')
            self.ax2.axhline(y=avg_w_csv, color='blue', linestyle=':', linewidth=2, label=f'CSV Avg: {avg_w_csv:.1f} kg')

            # Add a shaded region for the standard deviation of the live calculation
            self.ax2.axhspan(avg_w_calc - std_w_calc, avg_w_calc + std_w_calc, color='green', alpha=0.15, label=f'Live Calc Std Dev')

            self.ax2.set_title(f'Static Data Comparison: Live Calculation vs. CSV Data\n{stats_text}')
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
        self.time_fig.suptitle(f'Estimated Load vs. Time for "{self.current_filename}"')
        if 'time' in self.df.columns:
            self.ax_time.plot(self.df['time'], self.df[col_w_calculated], label='Live Calc Mass (w)')
            self.ax_time.plot(self.df['time'], self.df[col_w_csv], label='CSV Mass (w)', alpha=0.7, linestyle='--')

            file_index = FILENAMES.index(self.current_filename)
            actual_load = ACTUAL_LOAD[file_index]
            self.ax_time.axhline(y=actual_load, color='red', linestyle='--', linewidth=2, label=f'Actual: {actual_load} kg')

            self.ax_time.legend()
            self.ax_time.grid(True)
        else:
            self.ax_time.text(0.5, 0.5, "'__time' column not found in CSV.",
                              horizontalalignment='center', verticalalignment='center',
                              transform=self.ax_time.transAxes)

        self.ax_time.set_xlabel('Time (s)')
        self.ax_time.set_ylabel('Estimated Mass (w) [kg]')

        self.time_fig.canvas.draw_idle()
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

    def select_file(self, label):
        self.current_filename = label
        csv_file_path = os.path.expanduser(f'~/wheel_loader_ws/results/csv/{self.current_filename}.csv')

        try:
            df = pd.read_csv(csv_file_path)
            print(f"\nSuccessfully loaded '{csv_file_path}'")
        except FileNotFoundError:
            print(f"Error: The file '{csv_file_path}' was not found.")
            # Optionally, clear plots or show an error message on the plot
            return

        # --- Create relative time column ---
        if '__time' in df.columns:
            df['time'] = df['__time'] - df['__time'].iloc[0]

        # --- Handle potential missing velocity/acceleration data ---
        df[col_acc_g] = df[col_acc_g].fillna(0)
        df[col_vel_g] = df[col_vel_g].fillna(0)

        # Update the dataframe and replot
        self.df = df
        self.recalculate_and_plot()


def main():
    """
    Main function to load data, calculate mass, and plot results,
    simulating spreadsheet-like operations.
    """
    # --- Load Data ---
    initial_filename = FILENAMES[0]
    csv_file_path = os.path.expanduser(f'~/wheel_loader_ws/results/csv/{initial_filename}.csv')

    try:
        df = pd.read_csv(csv_file_path)
        print(f"Successfully loaded '{csv_file_path}'")
    except FileNotFoundError:
        print(f"Error: The file '{csv_file_path}' was not found.")
        return

    # --- Check for required columns ---
    required_cols = [col_theta_g, col_f, col_acc_g, col_vel_g, col_w_csv]
    if not all(col in df.columns for col in required_cols):
        print(f"Error: One or more required columns ({required_cols}) not found in the CSV file.")
        print(f"Available columns are: {df.columns.tolist()}")
        return

    # --- Create relative time column ---
    if '__time' in df.columns:
        df['time'] = df['__time'] - df['__time'].iloc[0]

    # --- Handle potential missing velocity/acceleration data ---
    # Assuming NaN means no movement, fill with 0.
    df[col_acc_g] = df[col_acc_g].fillna(0)
    df[col_vel_g] = df[col_vel_g].fillna(0)

    # --- Launch Interactive UI ---
    app = InteractiveTuner(df, initial_filename)
    plt.show()



if __name__ == '__main__':
    main()
