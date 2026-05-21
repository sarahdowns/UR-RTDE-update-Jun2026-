# File: move_insertion_test_from_cam_csv.py

import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import deque

from move_xyz_safe import UR5eSafeController
from gripper_RG2 import RG2

ip = "192.168.5.4"
rg_id = 0
rg_gripper = RG2(ip, rg_id)

ball_csv = "tennis_ball_xyz_20260428_122020.csv"
force_log_csv = "force_tracking_log.csv"

MOVE_SPEED = 0.20
MOVE_ACCEL = 1.0

APPROACH_OFFSET = 0.08   # 8 cm above target (industrial approach)
SETTLE_TIME = 0.6
RETRACT_HEIGHT = 0.10

MAX_FORCE = 50.0
CRITICAL_FORCE = 70.0
DEADBAND = 1.0

WINDOW = 150

# -----------------------------
# SAFETY STATE
# -----------------------------
class SafetyState:
    OK = 0
    WARNING = 1
    CRITICAL = 2
    SHUTDOWN = 3

# -----------------------------
# SAFETY EVALUATION
# -----------------------------
def evaluate_force(fx, fy, fz):
    max_f = max(abs(fx), abs(fy), abs(fz))

    if max_f > CRITICAL_FORCE:
        return SafetyState.CRITICAL
    elif max_f > MAX_FORCE:
        return SafetyState.WARNING
    else:
        return SafetyState.OK


# -----------------------------
# SHUTDOWN PROCEDURE
# -----------------------------
def safe_shutdown(robot, gripper):
    print("\n[SHUTDOWN] Force mode stop...")
    robot.rtde_c.forceModeStop()
    time.sleep(0.5)

    print("[SHUTDOWN] Opening RG2 gripper...")
    gripper.open()
    time.sleep(1.5)

    print("[SHUTDOWN] System safe.")
    robot.cleanup()


# -----------------------------
# MAIN
# -----------------------------
def main():

    print("[INIT] Loading dataset...")
    df = pd.read_csv(ball_csv)

    robot = UR5eSafeController(ip)
    
    gripper = RG2(ip, rg_id)
    print("[INIT] Setting RG2 initial grip state...")
    
    rg_gripper.rg_grip(70, 25.0)			# Fully open first (safe reset)
    time.sleep(2.0)
    
    rg_gripper.rg_grip(50, 25.0)	# Set initial controlled grip configuration
    time.sleep(1.0)
    print("[INIT] RG2 ready.")

    print("[INIT] Zeroing force sensor...")
    robot.zero_ft_sensor()
    time.sleep(1.0)

    # -------------------------
    # Logging
    # -------------------------
    logs = []

    # -------------------------
    # Live plot
    # -------------------------
    plt.ion()
    fig, ax = plt.subplots()

    fx_hist = deque(maxlen=WINDOW)
    fy_hist = deque(maxlen=WINDOW)
    fz_hist = deque(maxlen=WINDOW)

    lx, = ax.plot([], [], label="Fx")
    ly, = ax.plot([], [], label="Fy")
    lz, = ax.plot([], [], label="Fz")

    ax.set_ylim(-70, 70)
    ax.set_xlim(0, WINDOW)
    ax.legend()
    ax.grid(True)
    ax.set_title("Industrial Replay Force Monitoring")

    # -------------------------
    # CONTROL LOOP
    # -------------------------
    try:

        for i, row in df.iterrows():

            target = np.array([row["Ball_X"], row["Ball_Y"], row["Ball_Z"]])

            print(f"\n[POINT {i}] Target: {target}")

            # -------------------------
            # 1. APPROACH PHASE
            # -------------------------
            approach_target = target.copy()
            approach_target[2] += APPROACH_OFFSET

            ok = robot.move_to_xyz_safe(
                approach_target[0],
                approach_target[1],
                approach_target[2],
                visualize=False,
                speed=MOVE_SPEED,
                acceleration=MOVE_ACCEL,
                pitch_deg=180
            )

            if not ok:
                print("[SKIP] Approach failed")
                continue

            # -------------------------
            # 2. DESCEND PHASE (slow final contact)
            # -------------------------
            robot.move_to_xyz_safe(
                target[0], target[1], target[2],
                visualize=False,
                speed=0.10,
                acceleration=0.5,
                pitch_deg=180
            )

            # -------------------------
            # 3. SETTLE + MEASURE PHASE
            # -------------------------
            t0 = time.time()

            while time.time() - t0 < SETTLE_TIME:

                ft = robot.get_ft_sensor_baserot(world_frame=True)
                fx, fy, fz = ft[:3]
                tx, ty, tz = ft[3:]

                # deadband
                fx = 0 if abs(fx) < DEADBAND else fx
                fy = 0 if abs(fy) < DEADBAND else fy
                fz = 0 if abs(fz) < DEADBAND else fz

                # -------------------------
                # SAFETY SUPERVISOR
                # -------------------------
                state = evaluate_force(fx, fy, fz)

                if state == SafetyState.WARNING:
                    print("[WARNING] High force detected")

                if state == SafetyState.CRITICAL:
                    print("[CRITICAL] Force limit exceeded")

                    choice = input("Stop test and loosen gripper? (y/n): ")

                    if choice.lower() == "y":
                        safe_shutdown(robot, gripper)
                        return
                    else:
                        print("[INFO] Continuing with caution...")

                # -------------------------
                # LOGGING
                # -------------------------
                logs.append([
                    time.time(),
                    i,
                    *target,
                    fx, fy, fz,
                    tx, ty, tz
                ])

                # -------------------------
                # PLOT
                # -------------------------
                fx_hist.append(fx)
                fy_hist.append(fy)
                fz_hist.append(fz)

                x = range(len(fx_hist))
                lx.set_data(x, fx_hist)
                ly.set_data(x, fy_hist)
                lz.set_data(x, fz_hist)

                ax.set_xlim(0, WINDOW)

                plt.pause(0.01)

            # -------------------------
            # 4. RETRACT PHASE
            # -------------------------
            retract_target = target.copy()
            retract_target[2] += RETRACT_HEIGHT

            robot.move_to_xyz_safe(
                retract_target[0],
                retract_target[1],
                retract_target[2],
                visualize=False,
                speed=MOVE_SPEED,
                acceleration=MOVE_ACCEL,
                pitch_deg=180
            )

        print("\n[COMPLETE] All points processed")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Stopping safely...")

    finally:

        print("[SAVE] Writing logs...")
        out = pd.DataFrame(logs, columns=[
            "Time", "Index",
            "X", "Y", "Z",
            "Fx", "Fy", "Fz",
            "Tx", "Ty", "Tz"
        ])

        out.to_csv(insertion_force_log_csv, index=False)

        try:
            robot.rtde_c.forceModeStop()
        except:
            pass

        robot.cleanup()
        plt.ioff()
        plt.show()

        print("[DONE] Shutdown complete")


if __name__ == "__main__":
    main()
