# Octopus Pro arm serial GUI

The GUI scans USB serial ports and connects only when the firmware responds to
`INFO` with the Octopus protocol identifier. It sends poses in the existing
six-value format:

```text
POSE <base> <shoulder> <elbow> <wrist_pitch> <wrist_rotate> <gripper>
```

Install PySerial and launch the app from the repository root:

```bash
python3 -m pip install pyserial
python3 -m arm.arm_serial_gui
```

The STM32 firmware requires the Arduino libraries `AccelStepper` and
`TMCStepper`. The current base configuration uses the Octopus Pro `MOTOR0`
socket, with TMC2209 single-wire UART on `PC4`, a default run current of
1000 mA RMS, and a 300-1400 mA adjustment range. `UART_READBACK=ERROR` means
the driver register read failed; STEP/DIR motion may still work, and a current
write may still have been transmitted.

`Raw CW` and `Raw CCW` send direct diagnostic pulses at about 150 steps per
second without acceleration. Normal movement should use `POSE`, which applies
the configured acceleration profile.

The firmware's `ACTIVE_AXIS` array describes installed motors. A STEP/DIR
driver does not provide electrical motor-presence detection, so this list must
be updated and the firmware reflashed as motors are installed. The current
configuration enables only `BASE`; inactive controls are disabled in the GUI.

The arm must be physically placed at `HOME_ANGLE` before power-up. Endstop
homing is not implemented yet.
