#!/usr/bin/python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score

def main():
    """
    Performs polynomial regression on wheel loader sensor data to find the relationship
    between pressure sensor offsets and boom angle when the loader is static.
    This is used to model the pressure readings due to the linkage weight itself.
    """
    # --- Configuration ---
    csv_file_path = '/home/iwa/wheel_loader_ws/results/csv/all.csv'
    
    # Column names from the CSV file, as per note.md
    col_theta_g = '/debug_geometry/theta_g'
    col_median_pb = '/debug_geometry/median_pb'
    col_median_pr = '/debug_geometry/median_pr'
    col_omega_g = 'omega_g'
    col_omega_t = 'omega_t'
    
    # Static condition threshold (angular velocity close to zero)
    static_threshold = 0.001

    # --- Load Data ---
    try:
        df = pd.read_csv(csv_file_path)
    except FileNotFoundError:
        print(f"Error: The file '{csv_file_path}' was not found.")
        return

    # --- 1. Filter Data for Static Conditions ---
    # Extract data when the loader boom and tilt are not moving
    static_df = df[
        (df[col_omega_g].abs() < static_threshold) & 
        (df[col_omega_t].abs() < static_threshold)
    ].dropna(subset=[col_theta_g, col_median_pb, col_median_pr])

    if static_df.empty:
        print("No data found for the specified static conditions. Exiting.")
        return

    # Extract relevant columns for regression
    theta_g = static_df[col_theta_g]
    median_pb = static_df[col_median_pb]
    median_pr = static_df[col_median_pr]

    # --- 2. Regression for pb vs. theta_g (Polynomial Degree 2) ---
    print("--- Regression for Bottom Pressure (pb) vs. Boom Angle (theta_g) ---")
    
    # Fit a 2nd degree polynomial
    coeffs_pb = np.polyfit(theta_g, median_pb, 2)
    poly_pb = np.poly1d(coeffs_pb)
    
    # Calculate R^2 score
    pb_predicted = poly_pb(theta_g)
    r2_pb = r2_score(median_pb, pb_predicted)
    
    print(f"Function: pb(theta_g) = {coeffs_pb[0]:.4f}*theta_g^2 + {coeffs_pb[1]:.4f}*theta_g + {coeffs_pb[2]:.4f}")
    print(f"R^2 value: {r2_pb:.4f}\n")

    # --- 3. Regression for pr vs. theta_g (Polynomial Degree 1) ---
    print("--- Regression for Rod Pressure (pr) vs. Boom Angle (theta_g) ---")
    
    # Fit a 1st degree polynomial (linear)
    coeffs_pr = np.polyfit(theta_g, median_pr, 1)
    poly_pr = np.poly1d(coeffs_pr)
    
    # Calculate R^2 score
    pr_predicted = poly_pr(theta_g)
    r2_pr = r2_score(median_pr, pr_predicted)
    
    print(f"Function: pr(theta_g) = {coeffs_pr[0]:.4f}*theta_g + {coeffs_pr[1]:.4f}")
    print(f"R^2 value: {r2_pr:.4f}\n")

    # --- 4. Plotting ---
    print("--- Generating Plots ---")
    
    # Create a sorted sequence of theta_g for plotting the trend line
    theta_g_sorted = np.linspace(theta_g.min(), theta_g.max(), 100)
    pb_trend = poly_pb(theta_g_sorted)
    pr_trend = poly_pr(theta_g_sorted)

    # Plot 1: pb vs. theta_g
    plt.figure(figsize=(12, 6))
    plt.scatter(theta_g, median_pb, label='Static Data Points', alpha=0.5)
    plt.plot(theta_g_sorted, pb_trend, color='red', linewidth=2, label=f'Polynomial Fit (deg=2)\nR² = {r2_pb:.4f}')
    plt.title('Bottom Pressure (pb) vs. Boom Angle (theta_g) under Static Conditions')
    plt.xlabel('Boom Angle, theta_g (rad)')
    plt.ylabel('Median Bottom Pressure, pb (raw sensor value)')
    plt.legend()
    plt.grid(True)
    
    # Plot 2: pr vs. theta_g
    plt.figure(figsize=(12, 6))
    plt.scatter(theta_g, median_pr, label='Static Data Points', alpha=0.5, color='green')
    plt.plot(theta_g_sorted, pr_trend, color='red', linewidth=2, label=f'Linear Fit (deg=1)\nR² = {r2_pr:.4f}')
    plt.title('Rod Pressure (pr) vs. Boom Angle (theta_g) under Static Conditions')
    plt.xlabel('Boom Angle, theta_g (rad)')
    plt.ylabel('Median Rod Pressure, pr (raw sensor value)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()