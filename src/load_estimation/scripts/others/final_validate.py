#!/usr/bin/python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def main():
    """
    Analyzes and plots load estimation data from a CSV file.

    1. Extracts load (w) and boom angle (theta_g) when the loader is static.
    2. Creates a scatter plot of the extracted data.
    3. Calculates and displays the average and standard deviation of the load.
    4. Plots a line representing the actual load for comparison.
    2. Groups the static data by boom angle (theta_g) into bins.
    3. For each bin, calculates the average and standard deviation of the load (w).
    4. Creates an error bar plot of the binned data.
    5. Plots a line representing the actual load for comparison.
    """
    
    # --- Configuration ---
    filename = ['empty', 'twohuman', 'threebricks', 'sixbricks', 'ninebricks', 'tenbricks_singlehuman']
    csv_file_path = '~/wheel_loader_ws/results/csv/test5_empty_final.csv'
    csv_file_path = os.path.expanduser('~/wheel_loader_ws/results/csv/test5_empty_final.csv')
    
    # Column names from the CSV file
    col_w = '/debug/w'
    col_theta_g = '/debug/theta_g'
    col_acc_g = 'acc_g'
    col_vel_g = 'vel_g'
    
    # --- Thresholds and Constants ---
    # Thresholds to determine if the loader is in a static state
    ACC_THRESHOLD = 0.001  # rad/s^2
    VEL_THRESHOLD = 0.003  # rad/s
    
    # Set the actual load for the validation line 
    ACTUAL_LOAD = 0.0      # kg

    # Binning configuration for theta_g
    THETA_G_BIN_WIDTH = 0.05 # rad

    # --- Load Data ---
    try:
        df = pd.read_csv(csv_file_path)
    except FileNotFoundError:
        print(f"Error: The file '{csv_file_path}' was not found.")
        return

    # --- 1. Extract Static Data ---
    # Fill NaN values in velocity/acceleration columns with 0, assuming they represent no movement
    df[col_acc_g] = df[col_acc_g].fillna(0)
    df[col_vel_g] = df[col_vel_g].fillna(0)
    # Skip rows with NaN in velocity/acceleration

    # Filter the DataFrame for static conditions
    static_df = df[
        (df[col_acc_g].abs() < ACC_THRESHOLD) &
        (df[col_vel_g].abs() < VEL_THRESHOLD)
    ].dropna(subset=[col_w, col_theta_g])

    if static_df.empty:
        print("No data found for the specified static conditions. Cannot generate plot.")
        return

    w_values = static_df[col_w]
    theta_g_values = static_df[col_theta_g]
    # --- 2. Group data by theta_g bins ---
    theta_g_min = static_df[col_theta_g].min()
    theta_g_max = static_df[col_theta_g].max()

    # --- Calculate Statistics ---
    avg_w = w_values.mean()
    std_w = w_values.std()
    # Create bin edges
    bins = np.arange(theta_g_min, theta_g_max + THETA_G_BIN_WIDTH, THETA_G_BIN_WIDTH)
    
    # Lists to store binned statistics
    bin_centers = []
    avg_w_in_bin = []
    std_w_in_bin = []

    # print(f"--- Static Load Analysis ---")
    # print(f"Data points found: {len(w_values)}")
    # print(f"Average Load (w): {avg_w:.0f} kg")
    # print(f"Standard Deviation of Load (w): {std_w:.0f} kg")
    # --- 3. Calculate Statistics for each bin ---
    for i in range(len(bins) - 1):
        # Define the boundaries for the current bin
        bin_start = bins[i]
        bin_end = bins[i+1]
        
        # Filter data that falls into the current bin
        binned_data = static_df[
            (static_df[col_theta_g] >= bin_start) & 
            (static_df[col_theta_g] < bin_end)
        ]
        
        # If there's data in the bin, calculate stats
        if not binned_data.empty:
            bin_centers.append((bin_start + bin_end) / 2)
            avg_w_in_bin.append(binned_data[col_w].mean())
            std_w_in_bin.append(binned_data[col_w].std())

    # --- 2, 3, 4. Create Plot ---
    if not bin_centers:
        print("Could not create any data bins. Check data range and bin width.")
        return

    # --- 4 & 5. Create Plot ---
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.scatter(theta_g_values, w_values, alpha=0.6, label='Static Load Measurements')
    ax.errorbar(bin_centers, avg_w_in_bin, yerr=std_w_in_bin,
                fmt='o', capsize=5, alpha=0.7,
                label='empty (avg ± std)')
    ax.axhline(y=ACTUAL_LOAD, color='r', linestyle='--', linewidth=2, label=f'Actual Load = {ACTUAL_LOAD:.0f} kg')
    
    stats_text = f'Average Load: {avg_w:.0f} kg\nStd. Dev: {std_w:.0f} kg'
    ax.text(0.05, 0.95, stats_text, transform=ax.transAxes, fontsize=12,
            verticalalignment='top', bbox=dict(boxstyle='round,pad=0.5', fc='wheat', alpha=0.5))

    ax.set_xlabel('theta_g (rad)')
    ax.set_ylabel('load w (kg)')
    ax.set_title('Estimated Load (w) vs. Boom Angle (theta_g) under Static Conditions')
    ax.set_title('Estimated Load (w) vs. Boom Angle (theta_g)')
    ax.legend()
    ax.grid(True)
    plt.show()

if __name__ == '__main__':
    main()