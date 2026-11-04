# ESP32 + one Herkulex DRS-0601/DRS-0602

Minimal PlatformIO project for controlling one DRS-0601 or DRS-0602 from a standard ESP32
Dev Module. It uses hardware UART2 for the motor and UART0/USB for the serial
monitor.

## Wiring

| DRS-0602 signal | ESP32 Dev Module |
| --- | --- |
| Motor TX | GPIO16 (ESP32 RX) |
| Motor RX | GPIO17 (ESP32 TX) |
| GND | GND |

Power the DRS-0602 from a suitable external motor supply. Do not power it from
the ESP32. The motor supply and ESP32 must share ground. Verify that the motor's
serial logic levels are 3.3 V compatible before connecting it directly.

## Configuration

Edit `MOTOR_MODEL` in `include/BoardConfig.h` to select either
`MotorVariant::DRS_0601` or `MotorVariant::DRS_0602`. The same file contains the
UART pins and the initial `-30` to `+30` degree safety range. At startup the
firmware sends only `HSTAT` requests: it tries the configured baud rate and ID
first, then scans IDs 0-253 at every supported Herkulex baud rate. Register
reads and writes happen only after you enter their corresponding commands. A
complete unsuccessful scan takes roughly 15 seconds.

## Use

Build and upload:

```sh
pio run
pio run -t upload
pio device monitor
```

The firmware does not move the motor on startup. In the serial monitor, use:

```text
status
position
torque on
move 10
move -10
torque off
rescan
zeroall
```

`zeroall` is a connection diagnostic for DRS-0601/DRS-0602 motors whose ID is unknown.
It forces the motor UART to 115200 baud, broadcasts clear-error and torque-on,
then sends the selected model's 0-degree target to every valid motor ID, 0
through 253. It bypasses the normal online check and can move any connected
motor, so support the mechanism before using it.

Always test with the mechanism unloaded first. `torque off` releases the shaft;
support any load before using it.
