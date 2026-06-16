# Author: Sarah Downs
# File name: plot_master_dataset.py
# Description: Generates a comprehensive statistical dashboard from multiple 
# autonomous tactile insertion runs, including Mean and Std. Dev profiles.

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

CSV_FILE = "master_insertion_dataset_20cm.csv"

# --- Physical Ground Truths ---
SOCKET_SURFACE_Z = 0.122  
FORCE_LIMIT_Z = 20.0      
FORCE_LIMIT_XY = 20.0

# --- Depth Thresholds for Color Coding (in mm) ---
# Based on the 5mm (0.005m) socket depth from the robotic script
DEPTH_FULL_SUCCESS = 120    # >= 4.0 mm = Green (Full Insertion)
DEPTH_PARTIAL_SUCCESS = 50 # 1.0 to 4.0 mm = Yellow (Partial Entry / Wedged)
                            # < 1.0 mm = Red (Surface Jam)

def main():
    if not os.path.exists(CSV_FILE):
        print(f"[ERROR] '{CSV_FILE}' not found. Please run the insertion script and append data first.")
        return

    print(f"[INFO] Loading data from {CSV_FILE}...")
    df = pd.read_csv(CSV_FILE)

    if 'Test_ID' not in df.columns:
        print("[ERROR] 'Test_ID' column missing. Ensure you are using the updated master dataset script.")
        return

    test_ids = df['Test_ID'].unique()
    total_tests = len(test_ids)
    print(f"[INFO] Found {total_tests} individual test runs.")

    # Convert Z-coordinates to Relative Depth (mm)
    df['Depth_mm'] = (SOCKET_SURFACE_Z - df['World_Z'] - df['Rod_Length_m']) * 1000.0

    # ==========================================
    # SET UP 2x4 DASHBOARD GRID
    # ==========================================
    fig = plt.figure(figsize=(24, 12))
    gs = fig.add_gridspec(2, 4, width_ratios=[1, 1, 1, 0.85], wspace=0.25, hspace=0.3)
    
    # Row 0: Time Domain Plots
    ax_z_time  = fig.add_subplot(gs[0, 0])
    ax_x       = fig.add_subplot(gs[0, 1])
    ax_y       = fig.add_subplot(gs[0, 2])
    
    # Row 1: Spatial Domain Plots & Map
    ax_z_pos   = fig.add_subplot(gs[1, 0])
    ax_mean_z  = fig.add_subplot(gs[1, 1])
    ax_map     = fig.add_subplot(gs[1, 2])
    
    # Right Column: Text Report 
    ax_text    = fig.add_subplot(gs[:, 3])
    
    fig.suptitle("Tactile Insertion Analytics: Multi-Run Force Analysis", fontsize=22, fontweight='bold')

    full_count = 0
    part_count = 0
    jam_count = 0

    # ==========================================
    # PLOT LOOP: Individual Force Profiles
    # ==========================================
    for test_id in test_ids:
        test_data = df[df['Test_ID'] == test_id].copy()
        
        max_depth_mm = test_data['Depth_mm'].max()
        
        # Categorize Yield for Color Coding
        if max_depth_mm <= DEPTH_FULL_SUCCESS:
            line_color = '#2ca02c'  # Green
            alpha_val = 0.5
            full_count += 1
        elif max_depth_mm <= DEPTH_PARTIAL_SUCCESS:
            line_color = '#ffc107'  # Yellow/Gold
            alpha_val = 0.8
            part_count += 1
        else:
            line_color = '#d62728'  # Red
            alpha_val = 0.9 
            jam_count += 1
        
        # Row 0: Z, X, Y against TIME
        ax_z_time.plot(test_data['Time_s'], test_data['Force_Z_N'].abs(), color=line_color, alpha=alpha_val, linewidth=1.5)
        ax_x.plot(test_data['Time_s'], test_data['Force_X_N'], color=line_color, alpha=alpha_val, linewidth=1.5)
        ax_y.plot(test_data['Time_s'], test_data['Force_Y_N'], color=line_color, alpha=alpha_val, linewidth=1.5)

        # Row 1: Z-Force against Z-POSITION (Depth)
        ax_z_pos.plot(test_data['Depth_mm'], test_data['Force_Z_N'].abs(), color=line_color, alpha=alpha_val, linewidth=1.5)
        
        # Map: Plot the final XY resting position for this run
        final_pt = test_data.loc[test_data['Depth_mm'].idxmax()]
        ax_map.scatter(final_pt['World_X'], final_pt['World_Y'], color=line_color, 
                       marker='o', s=80, edgecolors='black', zorder=3, alpha=0.8)

    # ==========================================
    # MEAN AND STD DEV Z-FORCE CALCULATION
    # ==========================================
    # Define a common spatial grid to interpolate the data across
    min_depth = df['Depth_mm'].min()
    max_depth = df['Depth_mm'].max()
    depth_grid = np.linspace(min_depth, max_depth, 300) 
    
    interp_forces = []
    
    for test_id in test_ids:
        test_data = df[df['Test_ID'] == test_id].copy()
        # Group duplicates to ensure monotonic increasing depth for interpolation
        test_data = test_data.groupby('Depth_mm', as_index=False)['Force_Z_N'].mean()
        test_data = test_data.sort_values('Depth_mm')
        
        f_interp = np.interp(depth_grid, test_data['Depth_mm'], test_data['Force_Z_N'].abs())
        interp_forces.append(f_interp)
        
    interp_forces = np.array(interp_forces)
    mean_force = np.mean(interp_forces, axis=0)
    std_force = np.std(interp_forces, axis=0)
    
    # Plot Mean and Shaded Std Dev
    ax_mean_z.plot(depth_grid, mean_force, color='black', linewidth=2.5, label='Mean Z-Force')
    ax_mean_z.fill_between(depth_grid, np.maximum(0, mean_force - std_force), mean_force + std_force, 
                           color='#1f77b4', alpha=0.25, label='± 1 Std Deviation')

    ax_mean_z.axhline(y=FORCE_LIMIT_Z, color='red', linestyle='--', linewidth=1.5, label=f'Safety Limit ({FORCE_LIMIT_Z} N)')
    ax_mean_z.axvline(x=0, color='gray', linestyle='-', linewidth=1)
    
    ax_mean_z.set_title("Statistical Mean: Z-Force vs. Position", fontweight='bold')
    ax_mean_z.set_xlabel("Depth Below Socket Surface (mm)")
    ax_mean_z.set_ylabel("Absolute Z-Force (N)")
    ax_mean_z.grid(True, linestyle=':', alpha=0.7)
    ax_mean_z.legend(loc='upper left')

    # ==========================================
    # Formatting Time Domain Plots (Row 0)
    # ==========================================
    ax_z_time.set_title("Z-Axis Force vs. Time", fontweight='bold')
    ax_z_time.set_xlabel("Time (seconds)")
    ax_z_time.set_ylabel("Absolute Force (N)")
    ax_z_time.set_xlim(left=0)
    ax_z_time.grid(True, linestyle=':', alpha=0.7)
    
    custom_lines = [Line2D([0], [0], color='#2ca02c', lw=3),
                    Line2D([0], [0], color='#ffc107', lw=3),
                    Line2D([0], [0], color='#d62728', lw=3)]
    ax_z_time.legend(custom_lines, [
        f'Full Insertion (>= {DEPTH_FULL_SUCCESS} mm)', 
        f'Partial Wedge ({DEPTH_PARTIAL_SUCCESS}-{DEPTH_FULL_SUCCESS} mm)', 
        f'Surface Jam (< {DEPTH_PARTIAL_SUCCESS} mm)'
    ], loc='upper right', fontsize=9)

    ax_x.set_title("X-Axis Force vs. Time (Lateral)", fontweight='bold')
    ax_x.set_xlabel("Time (seconds)")
    ax_x.set_ylabel("Force (N)")
    ax_x.set_xlim(left=0)
    ax_x.axhline(y=FORCE_LIMIT_XY, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
    ax_x.axhline(y=-FORCE_LIMIT_XY, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
    ax_x.grid(True, linestyle=':', alpha=0.7)

    ax_y.set_title("Y-Axis Force vs. Time (Lateral)", fontweight='bold')
    ax_y.set_xlabel("Time (seconds)")
    ax_y.set_ylabel("Force (N)")
    ax_y.set_xlim(left=0)
    ax_y.axhline(y=FORCE_LIMIT_XY, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
    ax_y.axhline(y=-FORCE_LIMIT_XY, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
    ax_y.grid(True, linestyle=':', alpha=0.7)

    # ==========================================
    # Formatting Z-Force vs Position (Plot 4)
    # ==========================================
    ax_z_pos.axhline(y=FORCE_LIMIT_Z, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
    ax_z_pos.axvline(x=0, color='gray', linestyle='-', linewidth=1) 
    ax_z_pos.set_title("Z-Axis Force vs. Z-Position (All Runs)", fontweight='bold')
    ax_z_pos.set_xlabel("Depth Below Socket Surface (mm)")
    ax_z_pos.set_ylabel("Absolute Force (N)")
    ax_z_pos.grid(True, linestyle=':', alpha=0.7)

    # ==========================================
    # Formatting Target Map (Plot 6)
    # ==========================================
    ax_map.set_title("Final Resting Point Map (World XY)", fontweight='bold')
    ax_map.set_xlabel("World X (m)")
    ax_map.set_ylabel("World Y (m)")
    ax_map.grid(True, linestyle=':', alpha=0.7)
    
    # Force axes to maintain 1:1 physical aspect ratio if limits allow
    try:
        ax_map.axis('equal') 
    except:
        pass 

    # ==========================================
    # Text Box for Test Parameters
    # ==========================================
    ax_text.axis('off') 
    
    yield_rate = (full_count / total_tests) * 100 if total_tests > 0 else 0
    max_recorded_z = df['Force_Z_N'].abs().max()
    max_recorded_xy = max(df['Force_X_N'].abs().max(), df['Force_Y_N'].abs().max())
    
    report_text = (
        "DATASET OVERVIEW & RESULTS\n"
        "======================================\n\n"
        f"Total Insertions Logged: {total_tests}\n\n"
        "Mechanical Limits & Geometry:\n"
        f" • Z Runaway Limit: {FORCE_LIMIT_Z} N\n"
        f" • XY Runaway Limit: {FORCE_LIMIT_XY} N\n"
        f" • Socket Z-Ground Truth: {SOCKET_SURFACE_Z:.4f} m\n\n"
        "Yield Breakdown:\n"
        f" • Clean Entries (>= {DEPTH_FULL_SUCCESS}mm): {full_count}\n"
        f" • Frictional Jams ({DEPTH_PARTIAL_SUCCESS}-{DEPTH_FULL_SUCCESS}mm): {part_count}\n"
        f" • Surface Misses (< {DEPTH_PARTIAL_SUCCESS}mm): {jam_count}\n\n"
        "Peak Telemetry Across All Runs:\n"
        f" • Max Z-Force Experienced: {max_recorded_z:.2f} N\n"
        f" • Max XY-Force Experienced: {max_recorded_xy:.2f} N\n\n"
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
    output_png = "master_dataset_diagnostics.png"
    plt.savefig(output_png, dpi=300)
    print(f"[INFO] Diagnostic plot saved to: {output_png}")
    plt.show()

if __name__ == "__main__":
    main()
