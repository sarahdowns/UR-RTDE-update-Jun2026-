# Author: Sarah Downs
# File name: plot_insertion_algorithm_TRIALS.py
# This script generates a comprehensive kinematic and force dashboard 
# for a single autonomous tactile insertion sequence.

import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from matplotlib.lines import Line2D

#CSV_FILE = "full_insertion_sequence.csv"
CSV_FILE = "last_run_sequence.csv"

# --- Physical Ground Truths ---
SOCKET_SURFACE_Z = 0.122  # Ground truth from your main script
FORCE_LIMIT_Z = 20.0      # Safety limit from your compliance function
FORCE_LIMIT_XY = 20.0

def main():
    if not os.path.exists(CSV_FILE):
        print(f"[ERROR] '{CSV_FILE}' not found. Please run the insertion script first.")
        return

    print(f"[INFO] Loading data from {CSV_FILE}...")
    df = pd.read_csv(CSV_FILE)

    # --- Calculate Derived Metrics ---
    # Convert Z-coordinates to Relative Depth (mm)
    # 0 = Lip of the socket. Positive = Inside the socket. Negative = Hovering above.
    df['Depth_mm'] = (SOCKET_SURFACE_Z - df['World_Z']) * 1000.0
    
    phases = df['Phase'].unique()
    
    # Generate a distinct color map for the phases
    colors = plt.cm.tab10(np.linspace(0, 1, len(phases)))
    phase_colors = {phase: colors[i] for i, phase in enumerate(phases) if pd.notna(phase)}

    # SET UP 2x4 DASHBOARD GRID
    fig = plt.figure(figsize=(24, 12), layout="constrained")
    gs = gridspec.GridSpec(2, 4, width_ratios=[1, 1, 1, 0.85], wspace=0.25, hspace=0.3)
    
    # Row 0: Time Domain Plots
    ax_z_time  = fig.add_subplot(gs[0, 0])
    ax_x_time  = fig.add_subplot(gs[0, 1])
    ax_y_time  = fig.add_subplot(gs[0, 2])
    
    # Row 1: Spatial Domain Plots & Map
    ax_z_pos   = fig.add_subplot(gs[1, 0])
    ax_mag_pos = fig.add_subplot(gs[1, 1])
    ax_map     = fig.add_subplot(gs[1, 2])
    
    # Right Column: Text Report 
    ax_text    = fig.add_subplot(gs[:, 3])
    
    fig.suptitle("Tactile Insertion Fingerprint: Single Run Force Analysis", fontsize=20, fontweight='bold', y=0.96)

    # ==========================================
    # PLOT LOOP: Map data by Phase
    # ==========================================
    for phase in phases:
        if pd.isna(phase) or phase == "":
            continue
            
        p_data = df[df['Phase'] == phase]
        c = phase_colors[phase]
        
        # Row 0: Forces over Time
        ax_z_time.plot(p_data['Time_s'], p_data['Force_Z_N'].abs(), color=c, linewidth=2, label=phase)
        ax_x_time.plot(p_data['Time_s'], p_data['Force_X_N'], color=c, linewidth=2)
        ax_y_time.plot(p_data['Time_s'], p_data['Force_Y_N'], color=c, linewidth=2)

        # Row 1: Forces over Spatial Depth
        ax_z_pos.plot(p_data['Depth_mm'], p_data['Force_Z_N'].abs(), color=c, linewidth=2)
        ax_mag_pos.plot(p_data['Depth_mm'], p_data['Force_Mag_N'], color=c, linewidth=2)
        
        # Row 1: XY Kinematic Path
        # Use markers to show the density of data points (speed of motion)
        ax_map.plot(p_data['World_X'], p_data['World_Y'], color=c, linewidth=1.5, marker='.', markersize=4)

    # ==========================================
    # FORMATTING: Time Domain (Row 0)
    # ==========================================
    for ax, title, ylabel, limit in zip(
        [ax_z_time, ax_x_time, ax_y_time], 
        ["Absolute Z-Force vs. Time", "X-Axis Force vs. Time (Lateral)", "Y-Axis Force vs. Time (Lateral)"],
        ["Force (N)", "Force (N)", "Force (N)"],
        [FORCE_LIMIT_Z, FORCE_LIMIT_XY, FORCE_LIMIT_XY]
    ):
        ax.set_title(title, fontweight='bold')
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel(ylabel)
        ax.axhline(y=limit, color='red', linestyle='--', linewidth=1.5, alpha=0.7)
        if ax != ax_z_time: # X and Y can be negative
            ax.axhline(y=-limit, color='red', linestyle='--', linewidth=1.5, alpha=0.7)
        ax.grid(True, linestyle=':', alpha=0.7)

    ax_z_time.legend(loc='upper right', fontsize=9, framealpha=0.9)

    # ==========================================
    # FORMATTING: Spatial Domain (Row 1)
    # ==========================================
    for ax, title, ylabel in zip(
        [ax_z_pos, ax_mag_pos], 
        ["Absolute Z-Force vs. Insertion Depth", "Total Force Magnitude vs. Depth"],
        ["Z-Force (N)", "Total Force Vector (N)"]
    ):
        ax.set_title(title, fontweight='bold')
        ax.set_xlabel("Depth relative to Socket Surface (mm)\n[ Negative = Hovering | Positive = Inserted ]")
        ax.set_ylabel(ylabel)
        ax.axvline(x=0, color='black', linestyle='-', linewidth=1.5, alpha=0.8, label="Socket Surface")
        ax.grid(True, linestyle=':', alpha=0.7)
        
    ax_z_pos.axhline(y=FORCE_LIMIT_Z, color='red', linestyle='--', linewidth=1.5, alpha=0.7)
    
    # Map Formatting
    ax_map.set_title("XY Kinematic Path (World Frame)", fontweight='bold')
    ax_map.set_xlabel("World X (m)")
    ax_map.set_ylabel("World Y (m)")
    ax_map.grid(True, linestyle=':', alpha=0.7)
    ax_map.axis('equal') # Keep the physical aspect ratio 1:1

    # ==========================================
    # TEXT REPORT GENERATION
    # ==========================================
    ax_text.axis('off') 
    
    total_time = df['Time_s'].max()
    max_z_force = df['Force_Z_N'].abs().max()
    max_x_force = df['Force_X_N'].abs().max()
    max_y_force = df['Force_Y_N'].abs().max()
    max_depth_mm = df['Depth_mm'].max()
    
    # Extract the rod length safely if the column exists
    if 'Rod_Length_m' in df.columns:
        rod_length_mm = df['Rod_Length_m'].iloc[0] * 1000.0
        rod_text = f" • Calibrated Rod: {rod_length_mm:.1f} mm\n"
    else:
        rod_text = " • Calibrated Rod: Unknown\n"
    
    report_text = (
        "TACTILE SEQUENCE REPORT\n"
        "======================================\n\n"
        "Session Overview:\n"
        f" • Total Duration: {total_time:.2f} s\n"
        f" • Data Points Captured: {len(df)}\n"
        f"{rod_text}"
        #f" • Final Reached Depth: {max_depth_mm:.1f} mm\n\n"
        "Peak Force Telemetry:\n"
        f" • Max Z-Force: {max_z_force:.2f} N\n"
        f" • Max X-Force: {max_x_force:.2f} N\n"
        f" • Max Y-Force: {max_y_force:.2f} N\n\n"
        "--------------------------------------\n"
        "Algorithm Constraints:\n"
        f" • Z Runaway Limit: {FORCE_LIMIT_Z} N\n"
        f" • XY Runaway Limit: {FORCE_LIMIT_XY} N\n"
        "======================================"
    )
    
    ax_text.text(0.5, 0.5, report_text, fontsize=14, va='center', ha='center', 
                 bbox=dict(boxstyle='round,pad=1.5', facecolor='#f8f9fa', edgecolor='black', alpha=0.8),
                 family='monospace')

    # ==========================================
    # Save & Display
    # ==========================================
    output_png = "single_insertion_dashboard.png"
    plt.savefig(output_png, dpi=300)
    print(f"[INFO] Diagnostic plot saved to: {output_png}")
    plt.show()

if __name__ == "__main__":
    main()
