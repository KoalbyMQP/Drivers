import serial
import serial.tools.list_ports
import struct
import time

# --- Constants matching motorDefs.h / RPIComs.h ---
BUS_L_LEG_COUNT = 5
BUS_R_LEG_COUNT = 5
BUS_CHEST_COUNT = 5
BUS_L_ARM_COUNT = 5
BUS_R_ARM_COUNT = 5
TOTAL_COUNT     = BUS_L_LEG_COUNT + BUS_R_LEG_COUNT + BUS_CHEST_COUNT + BUS_L_ARM_COUNT + BUS_R_ARM_COUNT  # 25

NUM_INT16       = 1 + TOTAL_COUNT   # 26 (1 flag + 25 motors)
PACKET_SIZE     = NUM_INT16 * 2     # 52 bytes
START_BYTE      = 0xAA

BAUD_RATE       = 1000000
TIMEOUT_S       = 2.0

FLAG_START      =  1
FLAG_STOP       = -1
FLAG_CONTINUE   =  0

SCALE           = 100

MOTOR_LIMITS = [
    (-3.0,   19.0),
    (-16.0, 215.0),
    (-120.0, 33.0),
    (-69.0,  74.0),
    (-45.0,  42.0),
    (-19.0,   3.0),
    (-215.0, 16.0),
    (-33.0, 120.0),
    (-74.0,  69.0),
    (-42.0,  45.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
    (-45.0,   45.0),
    (-45.0,   45.0),
    (-40.0,   25.0),
    (-10.0,   10.0),
    (-40.0,  180.0),
    (-99.0,   80.0),
    (-180.0, 180.0),
    (-110.0, 110.0),
    (-45.0,   45.0),
    (-180.0,  40.0),
    (-80.0,   99.0),
    (-180.0, 180.0),
    (-110.0, 110.0),
]


def find_teensy_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "Teensy" in (p.description or "") or "usbmodem" in (p.device or ""):
            print(f"Auto-detected Teensy on {p.device}")
            return p.device
    print("Teensy not auto-detected. Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")
    idx = int(input("Select port index: "))
    return ports[idx].device


def build_packet(motor_targets, flag=FLAG_CONTINUE):
    targets = [max(lo, min(hi, v)) for v, (lo, hi) in zip(motor_targets, MOTOR_LIMITS)]
    while len(targets) < TOTAL_COUNT:
        targets.append(0.0)
    values  = [flag] + [int(v * SCALE) for v in targets]
    payload = struct.pack(f'<{NUM_INT16}h', *values)
    return bytes([START_BYTE]) + payload


def parse_response(line):
    try:
        values = [float(v.strip()) for v in line.strip().split(',') if v.strip()]
        return values if values else None
    except ValueError as e:
        print(f"  Parse error: {e} | raw: '{line.strip()}'")
        return None


def main():
    port = find_teensy_port()
    print(f"Connecting to {port} at {BAUD_RATE} baud...")

    with serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_S) as ser:
        ser.dtr = False
        time.sleep(3)
        ser.reset_input_buffer()
        print("Connected. Ctrl+C to stop.\n")

        # Temporarily in your loop, just to test:
        motor_targets = [float(i) for i in range(TOTAL_COUNT)]  # [0.0, 1.0, 2.0, ... 24.0]

        try:
            while True:
                # Send packet
                pkt = build_packet(motor_targets, flag=FLAG_CONTINUE)
                ser.write(pkt)
                ser.flush()  # ensure all bytes are sent before waiting

                # Wait for one complete response line
                raw = ser.readline()
                if not raw:
                    print("Timeout — no response.")
                    continue

                line = raw.decode('utf-8', errors='replace').strip()

                # Skip any debug print lines from Teensy
                if not line[0:1].lstrip('-').replace('.', '', 1).replace(',', '', 1).isdigit():
                    print(f"  Teensy debug: {line}")
                    continue

                values = parse_response(line)
                if values is not None:
                    print(f"Received ({len(values)} values): {values}")

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nSending stop packet...")
            ser.write(build_packet([0.0] * TOTAL_COUNT, flag=FLAG_STOP))
            print("Done.")

if __name__ == "__main__":
    main()