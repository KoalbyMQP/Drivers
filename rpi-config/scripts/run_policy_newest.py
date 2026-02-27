import torch
import numpy as np
import serial
import time
import signal
import sys
import json
import os

# CONFIGURATION
SERIAL_PORT = ""   # Pi GPIO UART: Identiy correct port
BAUDRATE = 115200              # Must match Arduino Mega Serial1
CONTROL_HZ = 100               # Policy updates per second
DT = 1.0 / CONTROL_HZ

POLICY_PATH = "/home/kfkartsen/avalocomotion/models/policy_ts.pt"
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
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=0.01)
    time.sleep(2)  # Give Arduino time to reset on serial connect

def init_policy():
    global policy, NUM_MOTORS
    policy = torch.jit.load(POLICY_PATH)
    policy.eval()

    # delete NUM_MOTORS from above when model fully set up
    with torch.no_grad():
        dummy_obs = torch.zeros(1, policy.graph.input_list()[0].type().sizes()[1])  # 1 x obs_dim
        dummy_action = policy(dummy_obs)
    NUM_MOTORS = dummy_action.numel()

# HELPER FUNCTIONS
def get_observation():
    """
    Collect sensor data from the robot.
    Return as a NumPy array matching training obs:
    [joint_pos, joint_vel, imu_orientation, imu_gyro, foot_contacts, velocity_command]
    
    TODO: implement exact reads from IMU, foot contacts, joint encoders.
    """
    # Example placeholders
    joint_pos = np.zeros(NUM_MOTORS)       # e.g., degrees or radians
    joint_vel = np.zeros(NUM_MOTORS)       # velocity of each joint
    imu_orientation = np.zeros(3)          # roll, pitch, yaw
    imu_gyro = np.zeros(3)                 # angular velocity x, y, z
    #foot_contacts = np.zeros(4)            # Not doing foot sensors
    velocity_command = np.zeros(3)         # target x, y, yaw

    obs = np.concatenate([
        joint_pos, joint_vel, imu_orientation, imu_gyro, velocity_command
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
    ser.write(csv_str.encode('utf-8'))

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
    global running

    init_serial()
    init_policy()
    print("Policy control started.")
    next_time = time.time()

    while running:
        try:

            # Read observations
            obs = get_observation()

            # Compute action from policy
            with torch.no_grad():
                action = policy(obs.unsqueeze(0)).squeeze(0).numpy()
                # action should be joint positions in degrees or radians (match training)

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
