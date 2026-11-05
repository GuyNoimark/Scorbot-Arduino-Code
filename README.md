# Scorbot Arduino Code

Firmware that brings an old **Scorbot robotic arm** back to life and lets you drive it with a **Logitech F310 gamepad**.

The arm sat unused in my high school's workshop for years, with no interface or manual, only a D-sub connector. During COVID I took it home, mapped every pin from the original datasheet, wired it to motor drivers and an Arduino Mega, and wrote this code to control it.

Full story with photos and videos: **[guynoimark.com/work/scorbot-robotics-project](https://guynoimark.com/work/scorbot-robotics-project/)**

## Hardware

- Scorbot robotic arm (base, two arm joints, gripper pitch, gripper roll and claw: 6 motors with quadrature encoders)
- Arduino Mega 2560
- USB Host Shield 2.0 (for the gamepad)
- Logitech F310 gamepad
- 3 motor driver boards (PWM + direction per motor)
- A power supply for the motors

## Wiring

Pins are set at the top of [`src/main.cpp`](src/main.cpp), in this order: base, arm, arm2, gripper rotation 1, gripper rotation 2, claw.

| Signal | Pins |
| --- | --- |
| Motor PWM | 7, 6, 5, 4, 3, 2 |
| Motor direction | A0 to A5 (54 to 59) |
| Encoder channel A | 20, 24, 26, 28, 30, 32 |
| Encoder channel B | 21, 25, 27, 29, 31, 33 |

## Controls

| Input | Action |
| --- | --- |
| Left stick (Y) | First arm joint |
| Right stick (Rz) | Second arm joint |
| RT / LT | Rotate base |
| D-pad | Gripper pitch and roll |
| A / B | Open / close claw |
| Y | Run an automatic base turn to a set angle |

## Code structure

```
src/main.cpp            setup, gamepad loop, joint mapping
lib/Motor               one joint: encoder reading, joystick control, position control that slows near the target
lib/Gripper             the three gripper motors together
lib/lf310               Logitech F310 driver on top of USB Host Shield
lib/InverseKinematics   joint angles from an XYZ target (work in progress)
```

## Build and upload

This is a [PlatformIO](https://platformio.org/) project. Dependencies (Encoder, PID, USB Host Shield 2.0) install automatically.

```bash
pio run --target upload
pio device monitor -b 9600
```

The serial monitor prints whether the gamepad is connected and the base position.
