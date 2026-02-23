import torch
import numpy as np
import serial
import time

# CONFIGURATION
SERIAL_PORT = ""   # Pi GPIO UART: Identiy correct port
BAUDRATE = 115200              # Must match Arduino Mega Serial1
CONTROL_HZ = 100               # Policy updates per second
DT = 1.0 / CONTROL_HZ

POLICY_PATH = "/home/kfkartsen/avalocomotion/models/policy_ts.pt"

''' DELETE ONCE MODEL FUNCTIONALITY CONFIRMED
NUM_MOTORS = 27
# Arduino expects joint positions in CSV order matching IDs on Mega

JOINT_LIMITS = [
    (-8, 8),      # chestturn
    (-25, 25),    # waistlean
    (-20, 10),    # Abdominalcrunch
    (-15, 0),     # hiplift_left
    (0, 15),      # hiplift_right
    (-12, 12),    # hiprotate_left
    (-12, 12),    # hiprotate_right
    (-50, 30),    # thighlift_left
    (-50, 30),    # thighlift_right
    (0, 55),      # knee_left
    (0, 55),      # knee_right
    (-10, 8),     # ankle_left
    (-10, 8),     # ankle_right
    (-20, 20),    # neckturn
    (-5, 10),     # headnod
    (-60, 60),    # shoulderspin_left
    (-60, 60),    # shoulderspin_right
    (-50, 80),    # bicep_left
    (-50, 80),    # bicep_right
    (-90, 90),    # elbow_left
    (-90, 90),    # elbow_right
    (-60, 60),    # wristspin_left
    (-60, 60),    # wristspin_right
    (-90, 90),    # handcurl_left
    (-90, 90),    # handcurl_right
    (-1, 0),      # gripper_left
    (-1, 0),      # gripper_right
]
'''
phase = 0.0
PHASE_SPEED = 1.5  # rad/sec

# INITIALIZE SERIAL
ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=0.01)
time.sleep(2)  # Give Arduino time to reset on serial connect

# LOAD POLICY
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

''' DELETE ONCE MODEL FUNCTIONALITY CONFIRMED
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
'''

# MAIN CONTROL LOOP
print("Policy control started.")
try:
    while True:
        t0 = time.time()

        # Read observations
        obs = get_observation()

        # Compute action from policy
        with torch.no_grad():
            action = policy(obs.unsqueeze(0)).squeeze(0).numpy()
            # action should be joint positions in degrees or radians (match training)
        
        '''DELETE ONCE MODEL FUNCTIONALITY CONFIRMED
        action = generate_placeholder_action()'''

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

