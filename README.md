# TPUP — SpotMicro Quadruped Firmware

Firmware for a 12-servo SpotMicro-style quadruped robot, written in C++ on an ESP32-S3 (ESP-IDF + FreeRTOS).

<!-- TODO: add a photo or GIF of the robot walking here -->

## What it does

- **Gait generation:** a trot gait built from trapezoidal foot paths, with a parametrized body height, step height, stride length, speed and duty factor.
- **Inverse kinematics:** closed-form, 3-DOF per-leg IK, verified across millions of sampled points. Unreachable targets fall back to fully extended or fully folded poses.
- **Body ↔ leg transforms:** foot targets in the body frame are mapped into each leg's frame.
- **Servo driver:** PCA9685 over I2C, with per-servo center calibration and per-joint angle limits.
- **IMU:** MPU-6050 driver with bias calibration, plus a complementary filter for orientation.
- **Control loop:** fixed-rate loop at ~83 Hz using `vTaskDelayUntil`.

## Hardware

| Part | Role |
|---|---|
| ESP32-S3-DevKitC-1 (8 MB flash) | Main controller |
| PCA9685 | 16-channel PWM servo driver |
| MPU-6050 | 6-axis IMU |
| 12× hobby servos | 3 per leg (hip, shoulder, knee) |

The I2C bus runs on SDA = GPIO 11, SCL = GPIO 12.

## Code map

| Module | Files |
|---|---|
| Gait | `Gait_Controller`, `TrapezoidPath` |
| Kinematics | `IK`, `Transformer`, `Spot_Model` |
| Hardware | `Servo`, `IMU`, `Common` (I2C setup and shared types) |
| Estimation | `Filter`, `State_Solver` (planned) |
| Testing | `Test_Session`, `Self_Test` |

Data flow: `GaitController → Transformer → IK → SpotModel → Servos`, with `IMU → Filter` feeding orientation back into `SpotModel`.

## Build & run

The build uses PlatformIO and has two environments. Both compile from the same tree, and a build flag picks the entry point:

```sh
pio run -e walk -t upload -t monitor   # gait control loop (main.cpp)
pio run -e test -t upload -t monitor   # interactive hardware test session (main/main.cpp)
```

VS Code tasks for both environments are in `.vscode/tasks.json` (Ctrl+Shift+B).

### Host simulation

`sim/` contains shim headers that let the control loop compile and run on a PC, so you can inspect the gait and kinematics math without hardware:

```powershell
.\sim\build_sim.ps1 -Run
```

## Testing

- **Self tests:** pure-math checks of IK, Transformer and TrapezoidPath that run on boot.
- **Hardware test session:** an interactive serial walkthrough covering the IMU (configuration, saturation, calibration, six-face orientation) and the servos (sweeps, limit saturation, protractor-verified angles). Results are saved to NVS across reboots and logged in [`test_results.md`](test_results.md).
- **IK audit:** [`REVIEW_NOTES.md`](REVIEW_NOTES.md) is a write-up of four branch-condition bugs found in the IK and the sweep-testing strategy that caught them.

## Status

- [x] IK, transforms and trapezoid gait verified
- [x] Servo calibration and limit protection
- [x] IMU validated
- [x] Open-loop walking
- [ ] IMU feedback in the control loop
- [ ] State estimation (`State_Solver`)
