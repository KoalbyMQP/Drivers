import torch
import numpy as np
import serial
import time
import signal
import sys
import json
import os

# CONFIGURATION
SERIAL_PORT = ""   # Pi GPIO UART: run "ls /dev/tty*" to determine
BAUDRATE = 115200              # Must match Arduino Mega Serial1
CONTROL_HZ = 100               # Policy updates per second
DT = 1.0 / CONTROL_HZ

POLICY_PATH = "./policy_ts.pt"
HEALTH_PATH = "/tmp/locomotion_health.json" # Allows Zara OS to monitor container functioning

running = True
ser = None
policy = None
NUM_MOTORS = None

# SIGNAL HANDLER-- allows Zara OS to stop container
def handle_shutdown(signum, frame):
    global running
    print(f"Received signal {signum}. Shutting down.")
    running = False

signal.signal(signal.SIGTERM, handle_shutdown)
signal.signal(signal.SIGINT, handle_shutdown)

# INITIALIZATION
def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=0.01)
        time.sleep(2)  # Give Arduino time to reset on serial connect
        print(f"Serial connection established on {SERIAL_PORT} at {BAUDRATE} baud.")
    except Exception as e:
        print(f"Error initializing serial connection: {e}")
        sys.exit(1)

def init_policy():
    global policy, NUM_MOTORS, last_action
    policy = torch.jit.load(POLICY_PATH)
    policy.eval()

    with torch.no_grad():
        dummy_obs = torch.zeros(1, 93)  # 1 x obs_dim
        dummy_action = policy(dummy_obs)
        
    NUM_MOTORS = dummy_action.numel()
    print("NUM_MOTORS:", NUM_MOTORS)
    last_action = np.zeros(NUM_MOTORS)

# HELPER FUNCTIONS
def get_observation():
    """
    Collect sensor data from the robot.
    Return as a NumPy array matching training obs:
    [base_lin_vel (3), base_ang_vel (3), proj_gravity (3), velocity_command (3), joint_pos (27), joint_vel (27), last_action (27)] = 93-dim obs
    
    TODO: Replace zeros with real inputs
    """
    base_lin_vel = np.zeros(3)
    base_ang_vel = np.zeros(3)
    proj_gravity = np.array([0, 0, -1])
    velocity_command = np.zeros(3)
    joint_pos = np.zeros(NUM_MOTORS)
    joint_vel = np.zeros(NUM_MOTORS)

    obs = np.concatenate([
        base_lin_vel, base_ang_vel, proj_gravity, velocity_command, joint_pos, joint_vel, last_action
    ])
    return torch.tensor(obs, dtype=torch.float32)

def send_motor_commands(joint_targets):
    """
    Send joint positions to Arduino Mega via UART.
    Arduino expects CSV string with newline at end. 
    TODO: Update when this changes to Arduino expects byte stream of motor positions
    """
    if len(joint_targets) != NUM_MOTORS:
        raise ValueError(f"Expected {NUM_MOTORS} joint_targets, got {len(joint_targets)}")
    
    csv_str = ",".join(f"{j:.3f}" for j in joint_targets) + "\n"
    try:
        ser.write(csv_str.encode('utf-8'))
    except Exception as e:
        print(f"Error sending motor commands: {e}")

def send_hold_command():
    """"
    Sends current joint positions to Arduino Mega via UART.
    Safety measure-- prevents unstable torques if container crashes while robot is moving.
    TODO: Can update to move to standing position if standing joint positions given.
    """
    obs = get_observation()
    current_positions = obs[:NUM_MOTORS].numpy()
    send_motor_commands(current_positions)

def write_health():
    """
    Writes whether container is active to json file.
    Allows Zara OS to monitor health of container.
    """
    try:
        with open(HEALTH_PATH, "w") as f:
            json.dump({
                "alive": True,
                "timestamp": time.time()
            }, f)
    except:
        pass

# MAIN CONTROL LOOP
def run():
    global running, last_action

    init_serial()
    init_policy()
    print("Policy control started.")
    next_time = time.time()

    send_hold_command()
    time.sleep(1)

    while running:
        try:

            # Read observations
            obs = get_observation()
            assert obs.shape[0] == 93, f"Obs wrong shape: {obs.shape}"

            # Compute action from policy
            with torch.no_grad():
                action = policy(obs.unsqueeze(0)).squeeze(0).numpy()

            last_action = action.copy()
            
            # Send action to Arduino
            send_motor_commands(action)

            # Write health to health path
            write_health()

        except KeyboardInterrupt:
            print("Exiting control loop")
            ser.close()
            
        # Maintain control rate-- self-corrects for timing drifts
        next_time += DT
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Exit
    print("Policy control stopping.")
    send_hold_command()

    if ser is not None:
        ser.close()

    print("Shutdown completed.")

# MAIN ENTRY
if __name__ == "__main__":
    run()
