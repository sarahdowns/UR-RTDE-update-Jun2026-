# Author: Sarah Downs
# File name: delete_trials.py
# Description: An interactive CLI tool to view and safely remove specific 
# trial runs from the master insertion dataset.

import os
import pandas as pd
import shutil

CSV_FILE = "master_insertion_dataset_20cm.csv"
BACKUP_FILE = "master_insertion_dataset_20cm.csv.bak"

def main():
    print("==========================================")
    print("      DATASET TRIAL MANAGER MANAGER       ")
    print("==========================================")

    if not os.path.exists(CSV_FILE):
        print(f"[ERROR] '{CSV_FILE}' not found in the current directory.")
        return

    # Load the dataset
    print(f"[INFO] Loading {CSV_FILE}...")
    df = pd.read_csv(CSV_FILE)

    if 'Test_ID' not in df.columns:
        print("[ERROR] 'Test_ID' column not found in the dataset.")
        return

    # Get unique Test IDs and their point counts
    test_ids = df['Test_ID'].unique()
    
    if len(test_ids) == 0:
        print("[INFO] The dataset is currently empty.")
        return

    print("\nAvailable Trials:")
    print("------------------------------------------")
    for i, test_id in enumerate(test_ids):
        point_count = len(df[df['Test_ID'] == test_id])
        print(f" [{i}] {test_id} ({point_count} data points)")
    print("------------------------------------------")

    # Prompt user for selection
    user_input = input("\nEnter the number(s) of the trial(s) you want to delete (comma-separated) or 'q' to quit: ").strip().lower()

    if user_input == 'q':
        print("[INFO] Exiting without making changes.")
        return

    # Parse input
    try:
        selected_indices = [int(x.strip()) for x in user_input.split(',')]
    except ValueError:
        print("[ERROR] Invalid input. Please enter numbers separated by commas.")
        return

    # Validate indices and collect IDs to delete
    ids_to_delete = []
        if 0 <= idx < len(test_ids):
            ids_to_delete.append(test_ids[idx])
        else:
            print(f"[WARNING] Index {idx} is out of bounds. Skipping.")

    if not ids_to_delete:
        print("[INFO] No valid trials selected for deletion. Exiting.")
        return

    print(f"\nYou have selected the following trials for deletion:")
    for tid in ids_to_delete:
        print(f" - {tid}")
        
    confirm = input("\nAre you sure you want to permanently delete these? (y/n): ").strip().lower()
    
    if confirm in ['y', 'yes']:
        # Create a backup of the original file just in case
        shutil.copy2(CSV_FILE, BACKUP_FILE)
        print(f"\n[INFO] Backup created at '{BACKUP_FILE}'")

        # Filter out the deleted IDs
        original_len = len(df)
        df_cleaned = df[~df['Test_ID'].isin(ids_to_delete)]
        new_len = len(df_cleaned)
        
        # Save the cleaned dataset
        df_cleaned.to_csv(CSV_FILE, index=False)
        
        print(f"[SUCCESS] Deleted {len(ids_to_delete)} trial(s).")
        print(f"[SUCCESS] Removed {original_len - new_len} rows. The dataset now has {new_len} rows remaining.")
    else:
        print("[INFO] Deletion cancelled. Master dataset unchanged.")

if __name__ == "__main__":
    main()
