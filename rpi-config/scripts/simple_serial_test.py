# import torch
import numpy as np
import serial
import time

# CONFIGURATION
SERIAL_PORT = "/dev/ttyAMA0"   # Pi GPIO UART
BAUDRATE = 9600              # Must match Arduino Mega Serial1
CONTROL_HZ = 64               # Policy updates per second
DT = 1.0 / CONTROL_HZ

#POLICY_PATH = "/home/kfkartsen/avalocomotion/models/policy_ts.pt"

NUM_MOTORS = 2 #23?
# Arduino expects joint positions in CSV order matching IDs on Mega

JOINT_LIMITS = [
    (-30, 30),      # chestturn
    (-50, 50),    # waistlean
    # (-20, 10),    # Abdominalcrunch
    # (-15, 0),     # hiplift_left
    # (0, 15),      # hiplift_right
    # (-12, 12),    # hiprotate_left
    # (-12, 12),    # hiprotate_right
    # (-50, 30),    # thighlift_left
    # (-50, 30),    # thighlift_right
    # (0, 55),      # knee_left
    # (0, 55),      # knee_right
    # (-10, 8),     # ankle_left
    # (-10, 8),     # ankle_right
    # (-20, 20),    # neckturn
    # (-5, 10),     # headnod
    # (-60, 60),    # shoulderspin_left
    # (-60, 60),    # shoulderspin_right
    # (-50, 80),    # bicep_left
    # (-50, 80),    # bicep_right
    # (-90, 90),    # elbow_left
    # (-90, 90),    # elbow_right
    # (-60, 60),    # wristspin_left
    # (-60, 60),    # wristspin_right
    # (-90, 90),    # handcurl_left
    # (-90, 90),    # handcurl_right
    # (-1, 0),      # gripper_left
    # (-1, 0),      # gripper_right
]
phase = 0.0
PHASE_SPEED = 1.5  # rad/sec

# INITIALIZE SERIAL
ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=0.01)
time.sleep(2)  # Give Arduino time to reset on serial connect

def send_motor_commands(joint_targets):
    """
    Send joint positions to Arduino Mega via UART.
    Arduino expects CSV string with newline at end. 
    TODO: Update when this changes to Arduino expects byte stream of motor positions
    """
    if len(joint_targets) != NUM_MOTORS:
        raise ValueError(f"Expected {NUM_MOTORS} joint_targets, got {len(joint_targets)}")
    
    csv_str = ",".join(f"{j:.3f}" for j in joint_targets) + "\n"
    ser.write(csv_str.encode('utf-8'))

def generate_placeholder_action():
    """
    Generates smooth sinusoidal motion within each joint's limits.
    """
    global phase
    phase += PHASE_SPEED * DT

    action = []
    for i, (low, high) in enumerate(JOINT_LIMITS):
        center = (low + high) / 2.0
        amplitude = (high - low) / 2.0

        # Phase offset per joint so they don't all move together
        value = center + amplitude * np.sin(phase + i * 0.2)
        action.append(value)

    return np.array(action, dtype=np.float32)

def main():
    print("sending a serial command has started")
    try:
        while True:
            t0 = time.time()


            action = generate_placeholder_action()

            # Send action to Arduino
            send_motor_commands(action)

            # Maintain control rate
            elapsed = time.time() - t0
            sleep_time = DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Exiting control loop")
        ser.close()

if __name__ == "__main__":
    main()
