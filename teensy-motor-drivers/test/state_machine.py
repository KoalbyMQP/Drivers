import serial
import time
import threading
from queue import Queue

# state machine to test the entire robot system with the esp32 and the teensy over serial communication

# teensy receives and sends serial - port 15 (FINLEY - robot arm)
# esp32 receives and sends serial - port 7 (SWAPPING_STATION - tool changer)

# Elevator positions:
# LP1 - left position 1, LP2 - left position 2
# RP1 - right position 1, RP2 - right position 2

# End effector status:
# 0 = no end effector, 1 = end effector attached

# Swap sequence:
# 1. Check end effector status (E)
# 2. Move elevator to correct height, wait for MC
# 3. Move arm to position (2 chained movements), wait for M|COMPLETE each
# 4. Attach or detach the end effector
#    - Deposit: wait 2 seconds then verify end effector is empty, continue regardless
#    - Attach: wait for M|COMPLETE, wait 2 seconds then verify end effector is attached, continue regardless
# 5. Move arm back down (2 chained movements), wait for M|COMPLETE each
# 6. Move elevator back to home, wait for MC
# 7. Return to idle

class StateMachine:
    def __init__(self):
        self.state = "IDLE"
        self.input_queue = Queue()
        self.running = True

        try:
            print("Connecting to Teensy (COM15)...")
            self.finley = serial.Serial('COM15', 115200, timeout=1)
            time.sleep(1)
            print("Teensy connected!")
        except Exception as e:
            print(f"Failed to connect to Teensy: {e}")
            self.finley = None

        try:
            print("Connecting to ESP32 (COM7)...")
            self.swapping_station = serial.Serial('COM7', 115200, timeout=1)
            time.sleep(1)
            print("ESP32 connected!")
        except Exception as e:
            print(f"Failed to connect to ESP32: {e}")
            self.swapping_station = None

    def print_help(self):
        """Print available commands"""
        print("\n=== Available Commands ===")

        print("\nSwap Commands:")
        print("  SWAP|DL1  - Deposit current tool to left slot 1")
        print("  SWAP|DL2  - Deposit current tool to left slot 2")
        print("  SWAP|DR1  - Deposit current tool to right slot 1")
        print("  SWAP|DR2  - Deposit current tool to right slot 2")
        print("  SWAP|AL1  - Attach tool from left slot 1")
        print("  SWAP|AL2  - Attach tool from left slot 2")
        print("  SWAP|AR1  - Attach tool from right slot 1")
        print("  SWAP|AR2  - Attach tool from right slot 2")

        print("\nMotor Commands (send to Teensy):")
        print("  M<pos0>|<pos1>|...|<pos9>|<timing>  - Move motors (e.g., M0|50|10|40|-9999|-9999|-9999|-9999|-9999|-9999|2000)")
        print("  * Use -9999 for no movement on a motor")
        print("  * Last value is timing in milliseconds")

        print("\nStatus Commands (send to Teensy):")
        print("  E   - Check end effector status")

        print("\nOximeter Commands (send to Teensy):")
        print("  OX|<timing>  - Read oximeter for specified milliseconds (e.g., OX|5000)")

        print("\nOther Commands:")
        print("  help  - Show this help message")
        print("  exit  - Exit the program")
        print("=" * 25 + "\n")

    def validate_command(self, command):
        """Validate command format before sending"""
        command = command.strip()

        if command == "help":
            return True, None

        if command == "exit":
            return True, None

        # Swap command - format: SWAP|<action><side><slot>
        # action: D = deposit, A = attach
        # side: L = left, R = right
        # slot: 1 or 2
        if command.startswith("SWAP|"):
            action = command[5:8]
            if action in ["DL1", "DL2", "DR1", "DR2", "AL1", "AL2", "AR1", "AR2"]:
                return True, "swap"
            else:
                print(f"Invalid swap command: {action}. Use D/A + L/R + 1/2 (e.g. SWAP|DL1)")
                return False, None

        # Motor movement command
        if command.startswith("M"):
            try:
                parts = command[1:].split('|')
                if len(parts) != 11:  # 10 positions + 1 timing
                    raise ValueError("Motor command must have 10 positions and 1 timing value")
                for i, part in enumerate(parts[:-1]):
                    val = float(part)
                    if val != -9999 and (val < -180 or val > 180):
                        raise ValueError(f"Position {i} out of range (-180 to 180)")
                timing = int(parts[-1])
                return True, "finley"
            except Exception as e:
                print(f"Invalid motor command: {e}")
                return False, None

        # Tool changer commands
        if command in ["DL", "DR", "AL", "AR", "LL", "LR"]:
            return True, "finley"

        # Status command
        if command == "E":
            return True, "finley"

        # Oximeter command - format: OX|<timing>
        if command.startswith("OX|"):
            try:
                timing = int(command[3:])
                if timing <= 0:
                    raise ValueError("Timing must be greater than 0")
                return True, "finley"
            except Exception as e:
                print(f"Invalid oximeter command: {e}")
                return False, None

        print(f"Unknown command: {command}")
        return False, None

    def send_command(self, command, target):
        """Send command to specified target device"""
        if target == "finley" and self.finley:
            self.finley.write(f"{command}\n".encode())
            print(f"  [SENT to Teensy] {command}")
        elif target == "swapping_station" and self.swapping_station:
            self.swapping_station.write(f"{command}\n".encode())
            print(f"  [SENT to ESP32] {command}")
        else:
            print(f"  [ERROR] {target} device not connected")

    def wait_for_response(self, device, expected, timeout=20):
        """Wait for a specific response from a device"""
        start = time.time()
        while time.time() - start < timeout:
            if device.in_waiting > 0:
                response = device.readline().decode().strip()
                if response:
                    print(f"  [Response] {response}")
                    if response == expected:
                        return True
            time.sleep(0.05)
        print(f"  [Timeout] Expected '{expected}' but got no response within {timeout}s")
        return False

    def check_end_effector(self):
        """Send E to Teensy and return (right_attached, left_attached) as bools"""
        print("  [CHECK] Reading end effector status...")
        self.finley.write("E\n".encode())
        start = time.time()
        while time.time() - start < 5:
            if self.finley.in_waiting > 0:
                response = self.finley.readline().decode().strip()
                if response.startswith("E|"):
                    parts = response.split("|")
                    if len(parts) == 3:
                        right_attached = int(parts[1]) != 0
                        left_attached  = int(parts[2]) != 0
                        print(f"  [Status] Right: {'ATTACHED' if right_attached else 'EMPTY'}, Left: {'ATTACHED' if left_attached else 'EMPTY'}")
                        return right_attached, left_attached
            time.sleep(0.05)
        print("  [ERROR] No end effector response received")
        return None, None

    def execute_swap(self, action_code):
        """Execute a full deposit or attach sequence"""
        action = action_code[0]   # D or A
        side   = action_code[1]   # L or R
        slot   = action_code[2]   # 1 or 2

        elevator_pos  = f"{side}P{slot}"   # e.g. LP1, RP2
        elevator_home = f"{side}P0"        # home position for that side e.g. LP0, RP0

        print(f"\n[SWAP] {'Depositing to' if action == 'D' else 'Attaching from'} {'left' if side == 'L' else 'right'} slot {slot}")
        print("-" * 40)

        # --- Step 1: Check end effector status ---
        print("\n[1/6] Checking end effector status...")
        right_attached, left_attached = self.check_end_effector()
        if right_attached is None:
            print("[ABORT] Could not read end effector status")
            self.state = "IDLE"
            return

        if action == "D":
            has_effector = right_attached if side == "R" else left_attached
            if not has_effector:
                print(f"[ABORT] No end effector on {'right' if side == 'R' else 'left'} side to deposit")
                self.state = "IDLE"
                return
        elif action == "A":
            has_effector = right_attached if side == "R" else left_attached
            if has_effector:
                print(f"[ABORT] End effector already attached on {'right' if side == 'R' else 'left'} side, deposit first")
                self.state = "IDLE"
                return

        # --- Step 2: Move elevator to position ---
        print(f"\n[2/6] Moving elevator to {elevator_pos}...")
        self.swapping_station.write(f"{elevator_pos}\n".encode())
        print(f"  [SENT to ESP32] {elevator_pos}")
        if not self.wait_for_response(self.swapping_station, "MC"):
            print("[ABORT] Elevator did not reach position")
            self.state = "IDLE"
            return

        # --- Step 3: Move arm to swap position (2 chained movements, left and right are different) ---
        print(f"\n[3/6] Moving arm to swap position...")

        if side == "L":
            # TODO: Replace with real motor positions for approaching the LEFT swap station
            approach_move_1 = "M70|-9999|30|-95|15|-90|-9999|-9999|-9999|-9999|5000"
            approach_move_2 = "M15|-9999|60|-95|8|-15|-9999|-9999|-9999|-9999|5000"
            approach_move_3 = "M-50|-9999|45|-10|8|-35|-9999|-9999|-9999|-9999|5000"

            # 
        else:
            # TODO: Replace with real motor positions for approaching the RIGHT swap station
            approach_move_1 = "M-10|-20|-9999|-9999|-9999|-9999|-9999|-9999|-9999|-9999|1500"
            approach_move_2 = "M-15|-25|-30|-9999|-9999|-9999|-9999|-9999|-9999|-9999|1500"
            approach_move_3 = "M-15|-25|-30|-9999|-9999|-9999|-9999|-9999|-9999|-9999|1500"

        print("  [3a] Approach movement 1...")
        self.finley.write(f"{approach_move_1}\n".encode())
        print(f"  [SENT to Teensy] {approach_move_1}")
        if not self.wait_for_response(self.finley, "M|COMPLETE"):
            print("[ABORT] Arm approach movement 1 did not complete")
            self.state = "IDLE"
            return

        print("  [3b] Approach movement 2...")
        self.finley.write(f"{approach_move_2}\n".encode())
        print(f"  [SENT to Teensy] {approach_move_2}")
        if not self.wait_for_response(self.finley, "M|COMPLETE"):
            print("[ABORT] Arm approach movement 2 did not complete")
            self.state = "IDLE"
            return
        
        print("  [3c] Approach movement 3 (final alignment)...")
        self.finley.write(f"{approach_move_3}\n".encode())
        print(f"  [SENT to Teensy] {approach_move_3}")
        if not self.wait_for_response(self.finley, "M|COMPLETE"):
            print("[ABORT] Arm approach movement 3 did not complete")
            self.state = "IDLE"
            return

        # --- Step 4: Attach or deposit the end effector ---
        teensy_cmd = f"{'A' if action == 'A' else 'D'}{side}"  # e.g. AL, DR
        print(f"\n[4/6] {'Attaching' if action == 'A' else 'Depositing'} end effector ({teensy_cmd})...")
        self.finley.write(f"{teensy_cmd}\n".encode())
        print(f"  [SENT to Teensy] {teensy_cmd}")

        if action == "D":
            # Wait 2 seconds for the deposit to settle then check, continue regardless
            print("  [WAIT] Waiting 2 seconds for deposit to settle...")
            time.sleep(2)
            print("  [CHECK] Verifying end effector released...")
            right_attached, left_attached = self.check_end_effector()
            if right_attached is None:
                print("  [WARN] Could not read end effector status after deposit, continuing anyway")
            else:
                attached = right_attached if side == "R" else left_attached
                if attached:
                    print("  [WARN] End effector may not have released, continuing anyway")
                else:
                    print("  [OK] End effector successfully released")

        elif action == "A":
            # Wait for M|COMPLETE, then wait 2 seconds and verify, continue regardless
            if not self.wait_for_response(self.finley, "M|COMPLETE"):
                print("  [WARN] Attach M|COMPLETE not received, continuing anyway")
            print("  [WAIT] Waiting 2 seconds for attach to settle...")
            time.sleep(2)
            print("  [CHECK] Verifying end effector attached...")
            right_attached, left_attached = self.check_end_effector()
            if right_attached is None:
                print("  [WARN] Could not read end effector status after attach, continuing anyway")
            else:
                attached = right_attached if side == "R" else left_attached
                if not attached:
                    print("  [WARN] End effector may not have attached, continuing anyway")
                else:
                    print("  [OK] End effector successfully attached")

        # --- Step 5: Move arm back to home (2 chained movements, left and right are different) ---
        print(f"\n[5/6] Moving arm back to home position...")

        if side == "L":
            # TODO: Replace with real motor positions for retracting from the LEFT swap station
            retract_move_1 = "M15|-9999|55|-95|10|-15|-9999|-9999|-9999|-9999|5000"
            retract_move_2 = "M70|-9999|30|-95|15|-90|-9999|-9999|-9999|-9999|5000"
            retract_move_3 = "M0|-9999|0|0|0|0|-9999|-9999|-9999|-9999|5000"

            
        else:
            # TODO: Replace with real motor positions for retracting from the RIGHT swap station
            retract_move_1 = "M-15|-25|-30|-9999|-9999|-9999|-9999|-9999|-9999|-9999|1500"
            retract_move_2 = "M0|0|0|-9999|-9999|-9999|-9999|-9999|-9999|-9999|1500"
            retract_move_3 = "M10|20|9999|9999|9999|9999|9999|9999|9999|9999|1500"

        print("  [5a] Retract movement 1...")
        self.finley.write(f"{retract_move_1}\n".encode())
        print(f"  [SENT to Teensy] {retract_move_1}")
        if not self.wait_for_response(self.finley, "M|COMPLETE"):
            print("[ABORT] Arm retract movement 1 did not complete")
            self.state = "IDLE"
            return

        print("  [5b] Retract movement 2...")
        self.finley.write(f"{retract_move_2}\n".encode())
        print(f"  [SENT to Teensy] {retract_move_2}")
        if not self.wait_for_response(self.finley, "M|COMPLETE"):
            print("[ABORT] Arm retract movement 2 did not complete")
            self.state = "IDLE"
            return
        
        print("  [5c] Retract movement 3 (final alignment)...")
        self.finley.write(f"{retract_move_3}\n".encode())
        print(f"  [SENT to Teensy] {retract_move_3}")
        if not self.wait_for_response(self.finley, "M|COMPLETE"):
            print("[ABORT] Arm retract movement 3 did not complete")
            self.state = "IDLE"
            return

        # --- Step 6: Move elevator back to home ---
        print(f"\n[6/6] Moving elevator back to home ({elevator_home})...")
        self.swapping_station.write(f"{elevator_home}\n".encode())
        print(f"  [SENT to ESP32] {elevator_home}")
        if not self.wait_for_response(self.swapping_station, "MC"):
            print("[WARN] Elevator did not confirm return to home")

        print(f"\n[SWAP] {'Deposit' if action == 'D' else 'Attach'} complete, returning to idle")
        print("-" * 40)
        self.state = "IDLE"

    def input_thread_worker(self):
        """Background thread to read user input without blocking"""
        while self.running:
            try:
                user_input = input()
                self.input_queue.put(user_input)
            except EOFError:
                self.running = False
                break
            except:
                break

    def run(self):
        """Main run loop"""
        self.print_help()

        # Start background input thread
        input_thread = threading.Thread(target=self.input_thread_worker, daemon=True)
        input_thread.start()

        print("> ", end="", flush=True)

        try:
            while self.running:
                # Check for user input
                if not self.input_queue.empty():
                    user_input = self.input_queue.get().strip()

                    if not user_input:
                        print("> ", end="", flush=True)
                        continue

                    if user_input.lower() == "help":
                        self.print_help()
                    elif user_input.lower() == "exit":
                        print("Exiting...")
                        self.running = False
                        break
                    else:
                        valid, target = self.validate_command(user_input)
                        if valid and target:
                            if target == "swap":
                                self.state = "SWAPPING"
                                action_code = user_input[5:8]  # e.g. DL1, AR2
                                self.execute_swap(action_code)
                            else:
                                self.send_command(user_input, target)

                    print("> ", end="", flush=True)

                # Check for serial responses from Teensy
                if self.finley and self.finley.in_waiting > 0:
                    response = self.finley.readline().decode().strip()
                    if response:
                        if response.startswith("E|"):
                            parts = response.split("|")
                            if len(parts) == 3:
                                tcr_status = "ATTACHED" if parts[1] >= "1" else "DETACHED"
                                tcl_status = "ATTACHED" if parts[2] >= "1" else "DETACHED"
                                print(f"\n[Teensy] End Effector Status - Right: {tcr_status}, Left: {tcl_status}")
                            else:
                                print(f"\n[Teensy] {response}")
                        elif response.startswith("OX|"):
                            parts = response.split("|")
                            if len(parts) == 2:
                                bpm = parts[1]
                                if bpm == "ERROR":
                                    print(f"\n[Teensy] Oximeter Error - Sensor not detected")
                                else:
                                    print(f"\n[Teensy] Heart Rate: {bpm} BPM")
                            else:
                                print(f"\n[Teensy] {response}")
                        elif response == "M|COMPLETE":
                            print(f"\n[Teensy] Motor movement complete")
                        else:
                            print(f"\n[Teensy] {response}")
                        print("> ", end="", flush=True)

                # Check for serial responses from ESP32
                if self.swapping_station and self.swapping_station.in_waiting > 0:
                    response = self.swapping_station.readline().decode().strip()
                    if response:
                        print(f"\n[ESP32] {response}")
                        print("> ", end="", flush=True)

                time.sleep(0.05)

        except Exception as e:
            print(f"\nError: {e}")
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False
            if self.finley:
                self.finley.close()
            if self.swapping_station:
                self.swapping_station.close()

if __name__ == "__main__":
    machine = StateMachine()
    machine.run()