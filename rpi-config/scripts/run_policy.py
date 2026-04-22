import torch
import numpy as np
import serial
import time
import signal
import sys
import json
import os

# CONFIGURATION
SERIAL_PORT = "ttyAMA0" #/dev/pts/3"   # Pi GPIO UART ("/dev/pts/3" when testing)
BAUDRATE = 1000000                # Must match Teensy baudrate
CONTROL_HZ = 100               # Policy updates per second
TIMEOUT = 0.01
DT = 1.0 / CONTROL_HZ

POLICY_PATH = "./final_policy.pt"
HEALTH_PATH = "/tmp/locomotion_health.json" # Allows Zara OS to monitor container functioning

# define flag values
START = 1
CONTINUE = 0
STOP = -1

START_BYTE = 0xAA

running = False
ser = None
policy = None

# SIGNAL HANDLER-- allows Zara OS to stop container
# def handle_shutdown(signum, frame):
#     global running
#     print(f"Received signal {signum}. Shutting down.")
#     running = False

# signal.signal(signal.SIGTERM, handle_shutdown)
# signal.signal(signal.SIGINT, handle_shutdown)

# INITIALIZATION
def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        time.sleep(2)  # Give Teensy time to reset on serial connect
        print(f"Serial connection established on {SERIAL_PORT} at {BAUDRATE} baud.")
    except Exception as e:
        print(f"Error initializing serial connection: {e}")
        sys.exit(1)

def init_policy():
    global policy, NUM_MOTORS, last_action
    policy = torch.jit.load(POLICY_PATH)
    policy.eval()

    with torch.no_grad():
        dummy_obs = torch.zeros(1, 59)  # 1 x obs_dim
        dummy_action = policy(dummy_obs)

    NUM_MOTORS = dummy_action.numel()
    print("NUM_MOTORS:", NUM_MOTORS)
    last_action = np.zeros(NUM_MOTORS)

# HELPER FUNCTIONS
def read_serial():
    """
    Read from serial port and parse incoming data.
    """
    try:
        raw = ser.readline() # Read a line of data (until newline) from serial port
        if raw:
            line = raw.decode('utf-8').strip()
            # print(f"Raw: {raw} | Parsed: {line }")
            return line
        else:
            return None
    except Exception as e:
        print(f"Error reading from serial port: {e}")
        return None
    
def get_observation(test=False):
    """
    Collect sensor data from the robot.
    Return as a NumPy array matching training obs:
    [base_ang_vel (3), proj_gravity (3), velocity_command (3), joint_pos (25), last_action (25)] = 59-dim obs
    TODO: Format for compact data package
    """
    global last_action

    if test:
        # Return dummy observation for testing without serial input
        return torch.zeros(59)
    
    line = read_serial()
    if line is None:
        return None
    
    try:
        parts = line.split(",")
        if len(parts) < 34:
            print(f"Warning: Expected at least 34 data points, got {len(parts)}. Line: {line}")
            return None
        data = np.array([float(x) for x in parts])
    except:
        print(f"Error parsing line: {line}")
        return None

    base_ang_vel = data[0:3]
    # accel, pitch, yaw = data[57:60]  #just accelerometer (first one)
    # proj_gravity = accel
    proj_gravity = data[3:6]  # for now (See above two lines)
    velocity_command = data[6:9]
    joint_pos = data[9:34]

    obs = np.concatenate([
        base_ang_vel, proj_gravity, velocity_command, joint_pos, last_action
    ])
    return torch.from_numpy(obs).float()

def send_motor_commands(joint_targets, flag):
    """
    Scale joint targets x100, round, convert to int (e.g. 1.236 -> 123.6 -> 124. -> 124)
    NOTE: .5 rounds to nearest **even** int
    NOTE: int16 range is -32,768 to 32,767
    Send joint targets to Teensy via UART as a byte stream.
    Format: [flag (1), joint_targets (NUM_MOTORS)]
    """
    if len(joint_targets) != NUM_MOTORS:
        raise ValueError(f"Expected {NUM_MOTORS} joint_targets, got {len(joint_targets)}")
    
    joint_targets = np.clip(joint_targets, -327.68, 327.67)
    joint_targets = np.round(joint_targets * 100).astype(np.int16)
    flag_arr = np.array([flag], dtype=np.int16)
    commands = np.concatenate([flag_arr, joint_targets])
    packet = bytes([START_BYTE]) + commands.tobytes()

    try:
        ser.write(packet)   # Change to packet to include start byte if needed
    except Exception as e:
        print(f"Error sending motor commands: {e}")

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

def testing_results(n, time, pos):
    model_dur = []
    loop_dur = []
    model_tot = 0
    loop_tot = 0

    for i in range(n):
        m_dur = time[i][2] - time[i][1]
        model_dur.append(m_dur)
        l_dur = time[i][3] - time[i][0]
        loop_dur.append(l_dur)

    print("\nTime to query model:")
    for i in range(n):
        print(f"Loop {i+1}: {model_dur[i]}")
        model_tot += model_dur[i]
    print(f"Average time to query model: {model_tot / n}")
    print(f"Maximum time to query model: {max(model_dur)}")

    print("\nTime to run loop:")
    for i in range(n):
        print(f"Loop {i+1}: {loop_dur[i]}")
        loop_tot += loop_dur[i]
    print(f"Average time to run loop: {loop_tot / n}")
    print(f"Maximum time to run loop: {max(loop_dur)}")

    print("\nExpected pos:\tActual pos:")
    for i in range(9):
        print(f"{pos[i][0]}\n{pos[i][1]}")


# MAIN CONTROL LOOP
def run():
    global running, last_action

    init_serial()
    init_policy()
    print("Policy control started.")

    next_time = time.time()
    flag = START
    running = True

    time.sleep(1)

    while running:
        try:
            # Read observations
            obs = get_observation()
            if obs is not None:
                assert obs.shape[0] == 59, f"Obs wrong shape: {obs.shape}"

                # Compute action from policy
                with torch.no_grad():
                    action = policy(obs.unsqueeze(0)).squeeze(0).cpu().numpy().astype(np.float32)

                last_action = action

                # Send action to Teensy
                send_motor_commands(action, flag=flag)
                flag = CONTINUE

                # Write health to health path when zara os implemented
                # write_health()
            
        except KeyboardInterrupt:
            print("Exiting control loop")
            running = False

        # Maintain control rate-- self-corrects for timing drifts
        next_time = max(next_time + DT, time.time())
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Exit
    print("Policy control stopping.")
    send_motor_commands(np.zeros(NUM_MOTORS), flag=STOP)

    if ser is not None:
        ser.close()

    print("Shutdown completed.")

def run_test():
    global running, last_action

    use_serial = False

    if use_serial:
        init_serial()   #Don't initialize serial for testing without Teensy connected
    init_policy()
    print("Policy control started.")

    next_time = time.time()
    flag = START
    running = True
    time.sleep(1)

    i = 0
    time_test = []
    pos_test = []

    while running and i < 30:
        try:
            time_test.append([time.time()]) #start loop time

            # Read observations
            obs = get_observation(test=True)
            if obs is not None:
                assert obs.shape[0] == 59, f"Obs wrong shape: {obs.shape}"

                if i > 0:
                    pos_test[i-1].append([obs[9:34]]) #actual positions

                time_test[i].append(time.time()) #send to model time

                # Compute action from policy
                with torch.no_grad():
                    action = policy(obs.unsqueeze(0)).squeeze(0).cpu().numpy().astype(np.float32)

                time_test[i].append(time.time()) #receive from model time

                last_action = action
                pos_test.append([last_action[9:34]]) #instructed positions

                # Send action to Teensy
                if use_serial:
                    send_motor_commands(action, flag=flag)   #Don't send commands for testing without Teensy connected
                flag = CONTINUE

                # Write health to health path when zara os implemented
                # write_health()

        except KeyboardInterrupt:
            print("Exiting control loop")
            running = False

        # Maintain control rate-- self-corrects for timing drifts
        next_time = max(next_time + DT, time.time())
        sleep_time = next_time - time.time()
        time_test[i].append(time.time()) #end loop time
        if sleep_time > 0:
            time.sleep(sleep_time)

        i = i + 1

    # Exit
    print("Policy control stopping.")
    if use_serial:
        send_motor_commands(np.zeros(NUM_MOTORS), flag=STOP)   #Don't send commands for testing without Teensy connected

    testing_results(i, time_test, pos_test)

    if ser is not None:
        ser.close()

    print("Shutdown completed.")


# MAIN ENTRY
if __name__ == "__main__":
    run_test()
