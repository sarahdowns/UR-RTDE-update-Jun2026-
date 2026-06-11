# Author: Sarah Downs
# File name: plot_mc_results.py
# This script analyzes Monte Carlo insertion tests, mapping temporal and spatial force profiles.

import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from matplotlib.lines import Line2D

CSV_FILE = "monte_carlo_results.csv"

# --- Physical Ground Truths from Telemetry ---
SOCKET_SURFACE_HEIGHT = 0.1175  # Top rim of the PVC socket
TOTAL_TARGET_DEPTH = 0.13       # 13.0 cm total commanded stroke

# --- Kinematic Test Parameters (For Report Annotations) ---
TEST_SPEED = 0.3   # m/s
TEST_ACCEL = 1.5   # m/s^2
FORCE_LIMIT = 20.0 # N

# --- Depth Thresholds for Color Coding (in meters) ---
DEPTH_FULL_SUCCESS = 0.12     # >= 12 cm (120 mm) = Green (Full Insertion with 1cm grace buffer)
DEPTH_PARTIAL_SUCCESS = 0.02  # 2 cm to 12 cm (20 - 120 mm) = Yellow (Partial Entry)
                              # < 2 cm (20 mm) = Red (Surface Jam)

def main():
    if not os.path.exists(CSV_FILE):
        print(f"[ERROR] '{CSV_FILE}' not found. Please run the Monte Carlo test first.")
        return

    print(f"[INFO] Loading data from {CSV_FILE}...")
    df = pd.read_csv(CSV_FILE)

    test_ids = df['Test_ID'].unique()
    total_tests = len(test_ids)

    # Calculate global spatial variables needed for plotting
    df['Insertion_Depth_mm'] = (SOCKET_SURFACE_HEIGHT - df['TCP_Z']) * 1000.0
    df['Force_Total'] = np.sqrt(df['Force_X']**2 + df['Force_Y']**2 + df['Force_Z']**2)

    # ==========================================
    # SET UP 2x4 DASHBOARD GRID
    # ==========================================
    fig = plt.figure(figsize=(24, 12), layout="constrained")
    gs = gridspec.GridSpec(2, 4, width_ratios=[1, 1, 1, 0.85], wspace=0.25, hspace=0.3)
    
    # Row 0: Time Domain Plots
    ax_z_time  = fig.add_subplot(gs[0, 0])
    ax_x       = fig.add_subplot(gs[0, 1])
    ax_y       = fig.add_subplot(gs[0, 2])
    
    # Row 1: Spatial Domain Plots & Map
    ax_z_pos   = fig.add_subplot(gs[1, 0])
    ax_tot_pos = fig.add_subplot(gs[1, 1])
    ax_map     = fig.add_subplot(gs[1, 2])
    
    # Right Column: Text Report 
    ax_text    = fig.add_subplot(gs[:, 3])
    
    fig.suptitle("Monte Carlo Insertion Diagnostics: Temporal & Spatial Force Analysis", fontsize=20, fontweight='bold', y=0.96)

    # ==========================================
    # PLOT LOOP: Individual Force Profiles
    # ==========================================
    for test_id in test_ids:
        test_data = df[df['Test_ID'] == test_id].copy()
        
        lowest_z = test_data['TCP_Z'].min()
        max_insertion_m = SOCKET_SURFACE_HEIGHT - lowest_z
        
        if max_insertion_m >= DEPTH_FULL_SUCCESS:
            line_color = '#2ca02c'  # Green
            alpha_val = 0.5
        elif max_insertion_m >= DEPTH_PARTIAL_SUCCESS:
            line_color = '#ffc107'  # Yellow/Gold
            alpha_val = 0.8
        else:
            line_color = '#d62728'  # Red
            alpha_val = 0.9 
        
        # Plot Row 0: Z, X, Y against TIME
        ax_z_time.plot(test_data['Time_s'], test_data['Force_Z'].abs(), color=line_color, alpha=alpha_val, linewidth=1.5)
        ax_x.plot(test_data['Time_s'], test_data['Force_X'], color=line_color, alpha=alpha_val, linewidth=1.5)
        ax_y.plot(test_data['Time_s'], test_data['Force_Y'], color=line_color, alpha=alpha_val, linewidth=1.5)

        # Plot Row 1: Z-Force against Z-POSITION (Depth)
        ax_z_pos.plot(test_data['Insertion_Depth_mm'], test_data['Force_Z'].abs(), color=line_color, alpha=alpha_val, linewidth=1.5)

    # Re-calculate clean totals
    stats_df = df.groupby('Test_ID')['TCP_Z'].min().reset_index()
    stats_df['depth_mm'] = (SOCKET_SURFACE_HEIGHT - stats_df['TCP_Z']) * 1000
    
    full_count = len(stats_df[stats_df['depth_mm'] >= (DEPTH_FULL_SUCCESS * 1000)])
    part_count = len(stats_df[(stats_df['depth_mm'] >= (DEPTH_PARTIAL_SUCCESS * 1000)) & (stats_df['depth_mm'] < (DEPTH_FULL_SUCCESS * 1000))])
    jam_count = len(stats_df[stats_df['depth_mm'] < (DEPTH_PARTIAL_SUCCESS * 1000)])

    # ==========================================
    # MEAN TOTAL FORCE CALCULATION (Plot 5)
    # ==========================================
    min_z = df['Insertion_Depth_mm'].min()
    max_z = df['Insertion_Depth_mm'].max()
    z_grid = np.linspace(min_z, max_z, 300) 
    
    interp_forces = []
    
    for test_id in test_ids:
        test_data = df[df['Test_ID'] == test_id].copy()
        test_data = test_data.groupby('Insertion_Depth_mm')['Force_Total'].mean().reset_index()
        test_data = test_data.sort_values('Insertion_Depth_mm')
        
        f_interp = np.interp(z_grid, test_data['Insertion_Depth_mm'], test_data['Force_Total'])
        interp_forces.append(f_interp)
        
    interp_forces = np.array(interp_forces)
    mean_force = np.mean(interp_forces, axis=0)
    std_force = np.std(interp_forces, axis=0)
    
    ax_tot_pos.plot(z_grid, mean_force, color='black', linewidth=2.5, label='Mean Total Force')
    ax_tot_pos.fill_between(z_grid, np.maximum(0, mean_force - std_force), mean_force + std_force, 
                            color='blue', alpha=0.25, label='± 1 Std Deviation')

    ax_tot_pos.axhline(y=FORCE_LIMIT, color='red', linestyle='--', linewidth=1.5, label=f'Safety Limit ({FORCE_LIMIT} N)')
    ax_tot_pos.axvline(x=0, color='gray', linestyle='-', linewidth=1)
    
    ax_tot_pos.set_xlim(-10, (TOTAL_TARGET_DEPTH * 1000) + 5) # <--- ADD THIS LINE
    ax_tot_pos.set_title("Total Force Magnitude vs. Z-Position")
    ax_tot_pos.set_xlabel("Depth Below Socket Surface (mm)")
    ax_tot_pos.set_ylabel("Total Force (N)")
    ax_tot_pos.grid(True, linestyle=':', alpha=0.7)
    ax_tot_pos.legend()

    # ==========================================
    # Formatting Time Domain Plots (Row 0)
    # ==========================================
    ax_z_pos.set_xlim(-10, (TOTAL_TARGET_DEPTH * 1000) + 5) # <--- ADD THIS LINE
    ax_z_pos.set_title("Z-Axis Force vs. Z-Position")
    ax_z_time.set_xlabel("Time (seconds)")
    ax_z_time.set_ylabel("Absolute Force (N)")
    ax_z_time.set_xlim(left=0)
    ax_z_time.grid(True, linestyle=':', alpha=0.7)
    
    custom_lines = [Line2D([0], [0], color='#2ca02c', lw=3),
                    Line2D([0], [0], color='#ffc107', lw=3),
                    Line2D([0], [0], color='#d62728', lw=3)]
    ax_z_time.legend(custom_lines, [
        f'Full Insertion (>= {DEPTH_FULL_SUCCESS * 100:.0f} cm)', 
        f'Partial Entry ({DEPTH_PARTIAL_SUCCESS * 100:.0f}-{DEPTH_FULL_SUCCESS * 100:.0f} cm)', 
        f'Surface Jam (< {DEPTH_PARTIAL_SUCCESS * 100:.0f} cm)'
    ])

    ax_x.set_title("X-Axis Force vs. Time (Lateral)")
    ax_x.set_xlabel("Time (seconds)")
    ax_x.set_ylabel("Force (N)")
    ax_x.set_xlim(left=0)
    ax_x.grid(True, linestyle=':', alpha=0.7)

    ax_y.set_title("Y-Axis Force vs. Time (Lateral)")
    ax_y.set_xlabel("Time (seconds)")
    ax_y.set_ylabel("Force (N)")
    ax_y.set_xlim(left=0)
    ax_y.grid(True, linestyle=':', alpha=0.7)

    # ==========================================
    # Formatting Z-Force vs Position (Plot 4)
    # ==========================================
    ax_z_pos.axhline(y=FORCE_LIMIT, color='black', linestyle='--', linewidth=1.5, label=f'Safety Limit ({FORCE_LIMIT} N)')
    ax_z_pos.axvline(x=0, color='gray', linestyle='-', linewidth=1) 
    ax_z_pos.set_title("Z-Axis Force vs. Z-Position")
    ax_z_pos.set_xlabel("Depth Below Socket Surface (mm)")
    ax_z_pos.set_ylabel("Absolute Force (N)")
    ax_z_pos.grid(True, linestyle=':', alpha=0.7)
    ax_z_pos.legend()

    # ==========================================
    # XY Target Map (Plot 6)
    # ==========================================
    target_data = df.drop_duplicates(subset=['Test_ID']).copy()
    
    min_z_df = df.groupby('Test_ID')['TCP_Z'].min().reset_index()
    min_z_df.rename(columns={'TCP_Z': 'Lowest_Reached_Z'}, inplace=True)
    target_data = pd.merge(target_data, min_z_df, on='Test_ID')

    for _, row in target_data.iterrows():
        insertion_depth = SOCKET_SURFACE_HEIGHT - row['Lowest_Reached_Z']
        
        if insertion_depth >= DEPTH_FULL_SUCCESS:
            pt_color = '#2ca02c'
            marker_style = 'o' 
        elif insertion_depth >= DEPTH_PARTIAL_SUCCESS:
            pt_color = '#ffc107'
            marker_style = 's' 
        else:
            pt_color = '#d62728'
            marker_style = 'X' 
            
        ax_map.scatter(row['Target_X'], row['Target_Y'], color=pt_color, marker=marker_style, 
                       s=95, edgecolors='black', zorder=3)

    mean_x = target_data['Target_X'].mean()
    mean_y = target_data['Target_Y'].mean()
    ax_map.scatter(mean_x, mean_y, color='blue', marker='*', s=200, label='Camera Mean Center', zorder=4)

    ax_map.set_title("Socket Tolerance Map")
    ax_map.set_xlabel("Target X (m)")
    ax_map.set_ylabel("Target Y (m)")
    ax_map.grid(True, linestyle=':', alpha=0.7)
    ax_map.axis('equal') 
    
    ax_map.scatter([], [], color='#2ca02c', marker='o', edgecolors='black', label=f'Full Insertion ({full_count})')
    ax_map.scatter([], [], color='#ffc107', marker='s', edgecolors='black', label=f'Partial Wedge ({part_count})')
    ax_map.scatter([], [], color='#d62728', marker='X', edgecolors='black', label=f'Surface Jam ({jam_count})')
    ax_map.legend()

    # ==========================================
    # Text Box for Test Parameters
    # ==========================================
    ax_text.axis('off') 
    
    yield_rate = (full_count / total_tests) * 100 if total_tests > 0 else 0
    
    report_text = (
        "TEST PARAMETERS & RESULTS\n"
        "======================================\n\n"
        f"Total Insertions Attempted: {total_tests}\n\n"
        "Kinematic Settings:\n"
        f" • Tool Speed: {TEST_SPEED} m/s\n"
        f" • Tool Accel: {TEST_ACCEL} m/s²\n\n"
        "Mechanical Limits & Geometry:\n"
        f" • Safety Force Limit: {FORCE_LIMIT} N\n"
        f" • Target Stroke: {TOTAL_TARGET_DEPTH * 1000:.0f} mm\n"
        f" • Socket Z-Ground Truth: {SOCKET_SURFACE_HEIGHT:.4f} m\n\n"
        "Yield Breakdown:\n"
        f" • Clean Entries (>= {DEPTH_FULL_SUCCESS * 1000:.0f}mm): {full_count}\n"
        f" • Frictional Jams ({DEPTH_PARTIAL_SUCCESS * 1000:.0f}-{DEPTH_FULL_SUCCESS * 1000:.0f}mm): {part_count}\n"
        f" • Surface Misses (< {DEPTH_PARTIAL_SUCCESS * 1000:.0f}mm): {jam_count}\n\n"
        "--------------------------------------\n"
        f"SYSTEM RELIABILITY: {yield_rate:.1f}%\n"
        "======================================"
    )
    
    ax_text.text(0.5, 0.5, report_text, fontsize=15, va='center', ha='center', 
                 bbox=dict(boxstyle='round,pad=1.5', facecolor='#f8f9fa', edgecolor='black', alpha=0.8),
                 family='monospace')

    # ==========================================
    # Save & Display
    # ==========================================
    output_png = "monte_carlo_diagnostics.png"
    plt.savefig(output_png, dpi=300)
    print(f"[INFO] Diagnostic plot saved to: {output_png}")
    plt.show()

if __name__ == "__main__":
    main()
