#!/usr/bin/python3

import pandas as pd
import matplotlib.pyplot as plt

def main():
    """
    Calculates and prints statistical metrics for pressure data from a CSV file.
    """
    # --- File Path ---
    csv_file_path = '/home/iwa/wheel_loader_ws/results/csv/pressure_pure_static.csv'

    # --- Load Data ---
    try:
        df = pd.read_csv(csv_file_path)
    except FileNotFoundError:
        print(f"Error: The file '{csv_file_path}' was not found.")
        return

    # --- Columns to Analyze ---
    columns_to_analyze = {
        'raw_b': 'Raw [b]',
        'lpf_b': 'LPF [b]',
        'median_b': 'Median [b]',
        'raw_r': 'Raw [r]',
        'lpf_r': 'LPF [r]',
        'median_r': 'Median [r]',
    }

    print("--- Statistical Analysis of Pressure Data ---\n")

    stats = {
        'Max': [],
        'Mean': [],
        'Min': [],
        'Variance': []
    }
    labels = []

    # --- Calculate and Print Statistics ---
    for col, name in columns_to_analyze.items():
        if col in df.columns:
            print(f"Statistics for: {name} ({col})")
            max_val = df[col].max()
            mean_val = df[col].mean()
            min_val = df[col].min()
            var_val = df[col].var()
            
            print(f"  - Max:      {max_val:.4f}")
            print(f"  - Mean:     {mean_val:.4f}")
            print(f"  - Min:      {min_val:.4f}")
            print(f"  - Variance: {var_val:.4f}\n")

            stats['Max'].append(max_val)
            stats['Mean'].append(mean_val)
            stats['Min'].append(min_val)
            stats['Variance'].append(var_val)
            labels.append(name)
        else:
            print(f"Column '{col}' not found in the CSV file.\n")

    # --- Plotting ---
    print("--- Generating Plots ---")
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle('Statistical Comparison of Filters', fontsize=16)

    metrics_to_plot = ['Max', 'Mean', 'Min', 'Variance']
    flat_axes = axes.flatten()

    for i, metric in enumerate(metrics_to_plot):
        ax = flat_axes[i]
        bars = ax.bar(labels, stats[metric], color=['#1f77b4', '#ff7f0e', '#2ca02c', '#1f77b4', '#ff7f0e', '#2ca02c'])
        ax.set_title(metric)
        ax.grid(True, axis='y', linestyle='--', alpha=0.7)
        ax.set_ylabel('Value')
        # Set y-axis to log scale for Variance for better visualization
        if metric == 'Variance':
            ax.set_yscale('log')
            ax.set_ylabel('Value (log scale)')

        # Add data labels on top of bars
        for bar in bars:
            yval = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2.0, yval, f'{yval:.2f}', va='bottom', ha='center')

    # Rotate x-axis labels for better readability
    for ax in flat_axes:
        ax.tick_params(axis='x', rotation=30, labelsize=10)

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.show()

if __name__ == '__main__':
    main()