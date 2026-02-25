import serial
import time

# Connect to Arduino (change COM3 to your port)
arduino = serial.Serial('COM14', 9600, timeout=1)
time.sleep(2)

try:
    while True:
        print("\n--- Motor Commands ---")
        print("1. Move Motor 1 to angle")
        print("2. Move Motor 2 to angle")
        print("3. Custom command")
        print("4. Exit")
        
        choice = input("Enter choice (1-4): ")
        
        if choice == '1':
            angle = input("Enter angle for Motor 1: ")
            command = f"MOTOR1:{angle}"
            arduino.write((command + '\n').encode('utf-8'))
            print(f"Sent: {command}")
            
        elif choice == '2':
            angle = input("Enter angle for Motor 2: ")
            command = f"MOTOR2:{angle}"
            arduino.write((command + '\n').encode('utf-8'))
            print(f"Sent: {command}")
            
        elif choice == '3':
            command = input("Enter custom command: ")
            arduino.write((command + '\n').encode('utf-8'))
            print(f"Sent: {command}")
            
        elif choice == '4':
            break
        
        # Read any response
        time.sleep(0.2)
        while arduino.in_waiting:
            response = arduino.readline().decode('utf-8').strip()
            print(f"Arduino: {response}")
            
except KeyboardInterrupt:
    print("\nClosing connection")
finally:
    arduino.close()