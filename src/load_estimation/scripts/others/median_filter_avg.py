#!/usr/bin/python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    """
    Calculates and prints statistical metrics for pressure data from a CSV file.
    """
    # --- File Path ---
    csv_file_path = '/home/iwa/wheel_loader_ws/results/csv/final_offset_filtered.csv'

    # --- Load Data ---
    try:
        df = pd.read_csv(csv_file_path)
    except FileNotFoundError:
        print(f"Error: The file '{csv_file_path}' was not found.")
        return

    # --- Columns to Analyze ---
    columns_to_analyze = {
        'raw_b': 'Raw [b]',
        'median_b': 'Median [b]',
        'raw_r': 'Raw [r]',
        'median_r': 'Median [r]',
    }

    # --- Define Time Sections for Averaging ---
    # Each sub-list represents a [start_time, end_time] pair for a section.
    time_sections = [
        [0, 10], [25, 30], [70, 71],
        [105, 106], [141, 142], [174, 175],
        [212,213],[252,253],[290,291]
    ]

    print("--- Median Values & Variance for Time Sections ---\n")

    # Data structure for storing section statistics for plotting
    section_stats = {col: {'median': [], 'var': []} for col in columns_to_analyze.keys()}
    section_labels = []

    # Check if 'time' column exists in the dataframe
    if 'time' not in df.columns:
        print("Error: 'time' column not found in CSV. Cannot perform section analysis.\n")
        return
    else:
        for i, section in enumerate(time_sections):
            start_time, end_time = section
            print(f"Section {i+1} (Time: {start_time}s - {end_time}s):")
            section_labels.append(f"S{i+1}")

            # Filter the dataframe for the current time section
            section_df = df[(df['time'] >= start_time) & (df['time'] <= end_time)]

            if section_df.empty:
                print("  - No data in this time range.\n")
                # Append NaN to keep data aligned for plotting if a section is empty
                for col in columns_to_analyze.keys():
                    section_stats[col]['median'].append(np.nan)
                    section_stats[col]['var'].append(np.nan)
                continue

            # Calculate and print the mean for each column of interest
            for col, name in columns_to_analyze.items():
                if col in section_df.columns:
                    median_val = section_df[col].median()
                    var_val = section_df[col].var()
                    print(f"  - Median {name}: {median_val:.4f}")
                    print(f"  - Variance {name}: {var_val:.4f}")
                    section_stats[col]['median'].append(median_val)
                    section_stats[col]['var'].append(var_val)
            print("")  # Newline for readability

    # --- Plotting ---
    print("--- Generating Plots ---")

    if not section_labels:
        print("No data processed for any section. Skipping plot generation.")
        return

    fig, axes = plt.subplots(2, 1, figsize=(15, 12), sharex=True)
    fig.suptitle('Per-Section Comparison of Raw vs. Median Filter', fontsize=16)

    # Grouped bar chart settings
    x = np.arange(len(section_labels))
    width = 0.2  # the width of the bars

    # --- Subplot 1: Medians ---
    ax1 = axes[0]
    # Use different colors for 'b' and 'r' sensors, and alpha for raw vs filtered
    bars1 = ax1.bar(x - width*1.5, section_stats['raw_b']['median'], width, label='Raw [b]', color='#1f77b4', alpha=0.7)
    bars2 = ax1.bar(x - width*0.5, section_stats['median_b']['median'], width, label='Median [b]', color='#1f77b4')
    bars3 = ax1.bar(x + width*0.5, section_stats['raw_r']['median'], width, label='Raw [r]', color='#ff7f0e', alpha=0.7)
    bars4 = ax1.bar(x + width*1.5, section_stats['median_r']['median'], width, label='Median [r]', color='#ff7f0e')

    ax1.set_ylabel('Median Pressure')
    ax1.set_title('Comparison of Median Pressure Values per Section')
    ax1.legend()
    ax1.grid(True, axis='y', linestyle='--', alpha=0.7)

    # Add data labels on top of the bars for medians
    for bars in [bars1, bars2, bars3, bars4]:
        for bar in bars:
            yval = bar.get_height()
            if not np.isnan(yval):
                ax1.annotate(f'{yval:.1f}',
                             xy=(bar.get_x() + bar.get_width() / 2.0, yval),
                             xytext=(0, 2), textcoords='offset points',
                             ha='center', va='bottom', fontsize=7, rotation=90)

    # --- Subplot 2: Variances ---
    ax2 = axes[1]
    bars5 = ax2.bar(x - width*1.5, section_stats['raw_b']['var'], width, label='Raw [b]', color='#1f77b4', alpha=0.7)
    bars6 = ax2.bar(x - width*0.5, section_stats['median_b']['var'], width, label='Median [b]', color='#1f77b4')
    bars7 = ax2.bar(x + width*0.5, section_stats['raw_r']['var'], width, label='Raw [r]', color='#ff7f0e', alpha=0.7)
    bars8 = ax2.bar(x + width*1.5, section_stats['median_r']['var'], width, label='Median [r]', color='#ff7f0e')

    ax2.set_ylabel('Pressure Variance (log scale)')
    ax2.set_title('Comparison of Pressure Variance per Section')
    ax2.set_yscale('log')
    ax2.legend()
    ax2.grid(True, axis='y', linestyle='--', alpha=0.7)

    # Add data labels on top of the bars for variances
    for bars in [bars5, bars6, bars7, bars8]:
        for bar in bars:
            yval = bar.get_height()
            if not np.isnan(yval) and yval > 0:
                ax2.annotate(f'{yval:.2e}',
                             xy=(bar.get_x() + bar.get_width() / 2.0, yval),
                             xytext=(0, 2), textcoords='offset points',
                             ha='center', va='bottom', fontsize=7, rotation=90)

    # Set shared x-axis properties
    plt.xticks(x, section_labels)
    plt.xlabel('Time Section')

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.show()

if __name__ == '__main__':
    main()