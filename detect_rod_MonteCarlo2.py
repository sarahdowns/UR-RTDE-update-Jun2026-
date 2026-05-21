import pandas as pd
import numpy as np
import os

def calculate_sigma(csv_file):
    if not os.path.exists(csv_file):
        print(f"[ERROR] File not found: {csv_file}")
        return

    # Load the data into a pandas DataFrame
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"[ERROR] Failed to read CSV file: {e}")
        return

    # Calculate the standard deviation for each column
    sigma_x = df['Rod_X'].std()
    sigma_y = df['Rod_Y'].std()
    sigma_z = df['Rod_Z'].std()

    # Calculate the mean for each column for context
    mean_x = df['Rod_X'].mean()
    mean_y = df['Rod_Y'].mean()
    mean_z = df['Rod_Z'].mean()

    print("\n--- Simulation Data Analysis ---")
    print(f"Number of samples: {len(df)}")
    print(f"Mean Coordinates (m):")
    print(f"  X-mean: {mean_x:.4f}")
    print(f"  Y-mean: {mean_y:.4f}")
    print(f"  Z-mean: {mean_z:.4f}")
    print(f"\nStandard Deviations (Sigma) (m):")
    print(f"  Sigma_X: {sigma_x:.6f}")
    print(f"  Sigma_Y: {sigma_y:.6f}")
    print(f"  Sigma_Z: {sigma_z:.6f}")
    
    print("\n[INFO] These sigma values represent the camera's spatial measurement uncertainty.")

if __name__ == "__main__":
    # Replace with the name of the CSV file generated from the previous step
    generated_file = "zed_ur_black_rod_3000_frames_20250912_094534" # Your file name here
    calculate_sigma(generated_file)
