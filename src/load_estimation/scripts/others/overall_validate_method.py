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
    
    # Column names from the CSV file
    col_w = '/debug/w'
    col_theta_g = '/debug/theta_g'
    col_acc_g = 'acc_g'
    col_vel_g = 'vel_g'

    # --- Thresholds and Constants ---
    ACC_THRESHOLD = 0.001  # rad/s^2
    VEL_THRESHOLD = 0.003  # rad/s
    THETA_G_BIN_WIDTH = 0.1 # rad

    fig, ax = plt.subplots(figsize=(12, 8))

    for i in range(len(filename)):
        csv_file_path = os.path.expanduser(f'~/wheel_loader_ws/results/csv/test5_{filename[i]}_final.csv')
        try:
            df = pd.read_csv(csv_file_path)
        except FileNotFoundError:
            print(f"Error: The file '{csv_file_path}' was not found.")
            continue

        # --- 1. Extract Static Data ---
        # Skip rows with NaN in velocity/acceleration
        # Filter the DataFrame for static conditions
        static_df = df[
            (df[col_acc_g].abs() < ACC_THRESHOLD) &
            (df[col_vel_g].abs() < VEL_THRESHOLD)
        ].dropna(subset=[col_w, col_theta_g])

        if static_df.empty:
            print(f"No data found for the specified static conditions in '{csv_file_path}'. Cannot generate plot.")
            continue

        # --- 2. Group data by theta_g bins ---
        theta_g_min = static_df[col_theta_g].min()
        theta_g_max = static_df[col_theta_g].max()

        # Create bin edges
        bins = np.arange(theta_g_min, theta_g_max + THETA_G_BIN_WIDTH, THETA_G_BIN_WIDTH)
        
        # Lists to store binned statistics
        bin_centers = []
        avg_w_in_bin = []
        std_w_in_bin = []

        # --- 3. Calculate Statistics for each bin ---
        for j in range(len(bins) - 1):
            # Define the boundaries for the current bin
            bin_start = bins[j]
            bin_end = bins[j+1]
            
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

        if not bin_centers:
            print("Could not create any data bins. Check data range and bin width.")
            continue

        # --- 4. Plot series on the same chart ---
        # Plot the binned data and get the container for the plot elements
        errorbar_container = ax.errorbar(bin_centers, avg_w_in_bin, yerr=std_w_in_bin,
                                         fmt='o', capsize=5, alpha=0.7,
                                         label=f'{filename[i]} (avg ± std)')
        # Get the color of the plotted series to use for the 'actual load' line
        series_color = errorbar_container.lines[0].get_color()
        # Plot the actual load line with the same color, but without a separate legend entry
        ax.axhline(y=ACTUAL_LOAD[i], color=series_color, linestyle='--', linewidth=2)
    
    # --- 5. Finalize and show plot ---
    ax.set_xlabel('theta_g (rad)')
    ax.set_ylabel('load w (kg)')
    ax.set_title('Estimated Load (w) vs. Boom Angle (theta_g)')
    ax.legend()
    ax.grid(True)
    plt.show()

if __name__ == '__main__':
    main()