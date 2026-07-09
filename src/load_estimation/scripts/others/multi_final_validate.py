#!/usr/bin/python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def main():
    # --- Configuration ---
    filename = ['empty', 'twohuman2', 'threebricks', 'sixbricks2', 'ninebricks', 'tenbricks_singlehuman2']
    # Set the actual load (kg) for the validation line 
    ACTUAL_LOAD = [0, 150, 105, 210, 315, 425]    
    
    for i in range(len(filename)):

        csv_file_path = os.path.expanduser(f'~/wheel_loader_ws/results/csv/test5_{filename[i]}_final.csv')
        
        # Column names from the CSV file
        col_w = '/debug/w'
        col_theta_g = '/debug/theta_g'
        col_acc_g = 'acc_g'
        col_vel_g = 'vel_g'
        
        # --- Thresholds and Constants ---
        # Thresholds to determine if the loader is in a static state
        ACC_THRESHOLD = 0.001  # rad/s^2
        VEL_THRESHOLD = 0.003  # rad/s

        # --- Load Data ---
        try:
            df = pd.read_csv(csv_file_path)
        except FileNotFoundError:
            print(f"Error: The file '{csv_file_path}' was not found.")
            continue

        # --- 1. Extract Static Data ---
        # Filter the DataFrame for static conditions
        # Any rows with NaN in velocity/acceleration will be skipped because a boolean
        # condition involving NaN (e.g., `NaN < 0.001`) evaluates to False.
        static_df = df[
            (df[col_acc_g].abs() < ACC_THRESHOLD) &
            (df[col_vel_g].abs() < VEL_THRESHOLD)
        ].dropna(subset=[col_w, col_theta_g])

        if static_df.empty:
            print(f"No data found for the specified static conditions in '{csv_file_path}'. Cannot generate plot.")
            continue

        w_values = static_df[col_w]
        theta_g_values = static_df[col_theta_g]

        # --- Calculate Statistics ---
        avg_w = w_values.mean()
        std_w = w_values.std()

        # --- 2, 3, 4. Create Plot ---
        fig, ax = plt.subplots(figsize=(12, 8))
        ax.scatter(theta_g_values, w_values, alpha=0.6, label='Static Load Measurements')
        ax.axhline(y=ACTUAL_LOAD[i], color='r', linestyle='--', linewidth=2, label=f'Actual Load = {ACTUAL_LOAD[i]:.0f} kg')
        
        stats_text = f'Average Load: {avg_w:.0f} kg\nStd. Dev: {std_w:.0f} kg'
        ax.text(0.05, 0.95, stats_text, transform=ax.transAxes, fontsize=12,
                verticalalignment='top', bbox=dict(boxstyle='round,pad=0.5', fc='wheat', alpha=0.5))

        ax.set_xlabel('theta_g (rad)')
        ax.set_ylabel('load w (kg)')
        ax.set_title(f'Estimated Load (w) vs. Boom Angle (theta_g) under Static Conditions {filename[i]}')
        ax.legend()
        ax.grid(True)

    plt.show()

if __name__ == '__main__':
    main()