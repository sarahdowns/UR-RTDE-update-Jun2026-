# Author: Sarah Downs
# This code interprets a separate CSV file and plots TCP Z, Force X, Y, Z.

import csv
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def smooth(data, window=11, poly=2):
    return savgol_filter(data, window, poly) if len(data) > window else data

# Read the CSV file
timestamps, tcp_z, force_x, force_y, force_z = [], [], [], [], []

with open("robot_log.csv", newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        timestamps.append(float(row["timestamp"]))
        tcp_z.append(float(row["tcp_z"]))
        force_x.append(float(row["force_x"]))
        force_y.append(float(row["force_y"]))
        force_z.append(float(row["force_z"]))

window_size = 11  # Must be odd
poly_order = 2
if len(timestamps) >= window_size:
    tcp_z = savgol_filter(tcp_z, window_size, poly_order)
    force_x = savgol_filter(force_x, window_size, poly_order)
    force_y = savgol_filter(force_y, window_size, poly_order)
    force_z = savgol_filter(force_z, window_size, poly_order)

# Create 2x2 subplot
fig, axs = plt.subplots(2, 2, figsize=(6, 7))
ax1, ax2, ax3, ax4 = axs.flatten()

# Plot
ax1.plot(timestamps, tcp_z, label="TCP Z", color='blue')
ax2.plot(timestamps, force_x, label="Force X", color='green')
ax3.plot(timestamps, force_y, label="Force Y", color='red')
ax4.plot(timestamps, force_z, label="Force Z", color='orange')

# Labels and legends
ax1.set_ylabel("TCP Z (m)")
ax2.set_ylabel("Force X (N)")
ax3.set_ylabel("Force Y (N)")
ax4.set_ylabel("Force Z (N)")
for ax in [ax1, ax2, ax3, ax4]:
    ax.set_xlabel("Time (s)")
    ax.legend()
    ax.grid(True)

plt.tight_layout()
plt.show()