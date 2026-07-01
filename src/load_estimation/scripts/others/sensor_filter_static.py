#!/usr/bin/python3

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def lowpass_filter_series(series, alpha):
    """
    Applies a low-pass filter to a pandas Series.
    y[t] = alpha * x[t] + (1 - alpha) * y[t-1]
    """
    filtered_series = np.zeros_like(series)
    if len(series) > 0:
        filtered_series[0] = series.iloc[0]
        for i in range(1, len(series)):
            filtered_series[i] = alpha * series.iloc[i] + (1 - alpha) * filtered_series[i-1]
    return filtered_series

def median_filter_series(series, window_size):
    """
    Applies a median filter to a pandas Series using a rolling window.
    """
    return series.rolling(window=window_size, min_periods=1).median()

def main():
    # --- File Paths ---
    # Input CSV file path
    input_csv_path = '/home/iwa/wheel_loader_ws/results/csv/final_offset.csv'
    # Output CSV file path
    output_csv_path = '/home/iwa/wheel_loader_ws/results/csv/final_offset_filtered.csv'

    # --- Parameters ---
    alpha = 0.1  # Smoothing factor for low-pass filter
    window_size = 50  # Window size for median filter

    # --- Column Names from CSV ---
    time_col = '__time'
    pressure_b_col = '/vehicle/loader_joint_pressures/data[0]'
    pressure_r_col = '/vehicle/loader_joint_pressures/data[1]'
    angle_col = '/vehicle/loader_joint_angles/data[0]'

    # --- Load Data ---
    try:
        df = pd.read_csv(input_csv_path)
    except FileNotFoundError:
        print(f"Error: The file '{input_csv_path}' was not found.")
        print("Please check the 'input_csv_path' variable in the script.")
        return

    # --- Data Cleaning and Preparation ---
    # Forward-fill missing values to handle asynchronous logging
    df[pressure_b_col] = df[pressure_b_col].ffill()
    df[pressure_r_col] = df[pressure_r_col].ffill()
    df[angle_col] = df[angle_col].ffill()

    # Drop rows where sensor data is still NaN (i.e., at the beginning of the file)
    df.dropna(subset=[pressure_b_col, pressure_r_col, angle_col], inplace=True)

    # Ensure time starts from 0
    df['time'] = df[time_col] - df[time_col].iloc[0]
    
    # Convert angle to degrees
    df['lift_angle_deg'] = df[angle_col] * 180 / np.pi

    # --- Apply Filters ---
    # Low-pass filter
    df['lowpass_b'] = lowpass_filter_series(df[pressure_b_col], alpha)
    df['lowpass_r'] = lowpass_filter_series(df[pressure_r_col], alpha)

    # Median filter
    df['median_b'] = median_filter_series(df[pressure_b_col], window_size)
    df['median_r'] = median_filter_series(df[pressure_r_col], window_size)

    # --- Save Processed Data to CSV ---
    output_df = pd.DataFrame({
        'time': df['time'],
        'raw_b': df[pressure_b_col],
        'raw_r': df[pressure_r_col],
        'median_b': df['median_b'],
        'median_r': df['median_r'],
        'lpf_b': df['lowpass_b'],
        'lpf_r': df['lowpass_r'],
        'lift_angle(deg)': df['lift_angle_deg']
    })
    output_df.to_csv(output_csv_path, index=False, float_format='%.6f')
    print(f"Processed data saved to '{output_csv_path}'")

    # --- Plotting ---
    fig, axes = plt.subplots(3, 1, figsize=(15, 12), sharex=True)
    fig.suptitle('Pressure and Angle Data with Filters', fontsize=16)

    # Subplot 1: Pressure data[0]
    axes[0].plot(df['time'], df[pressure_b_col], label='raw', alpha=0.7)
    axes[0].plot(df['time'], df['lowpass_b'], label='low-pass filter', linewidth=2)
    axes[0].plot(df['time'], df['median_b'], label='median filter', linewidth=2)
    axes[0].set_ylabel('raw_b')
    axes[0].legend()
    axes[0].grid(True)
    axes[0].set_title('Pressure Data [0]')

    # Subplot 2: Pressure data[1]
    axes[1].plot(df['time'], df[pressure_r_col], label='raw', alpha=0.7)
    axes[1].plot(df['time'], df['lowpass_r'], label='low-pass filter', linewidth=2)
    axes[1].plot(df['time'], df['median_r'], label='median filter', linewidth=2)
    axes[1].set_ylabel('raw_r')
    axes[1].legend()
    axes[1].grid(True)
    axes[1].set_title('Pressure Data [1]')

    # Subplot 3: Angle data[0]
    axes[2].plot(df['time'], df['lift_angle_deg'], label='lift_angle(deg)')
    axes[2].set_ylabel('Angle (deg)')
    axes[2].set_title('Angle Data [0]')

    axes[2].set_xlabel('time (s)')
    axes[2].grid(True)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.show()


if __name__ == '__main__':
    # The argparse import is not used, but we keep it to avoid breaking the script if it's used elsewhere.
    main()