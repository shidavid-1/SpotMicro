# TPUP — SpotMicro Quadruped Firmware

Firmware for a 12-servo SpotMicro-style quadruped robot, written in C++ on an ESP32-S3 (ESP-IDF + FreeRTOS).

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

| Folder | Contents |
|---|---|
| [`main/`](main/) | Entry points: `walk_main.cpp` (control loop), `test_main.cpp` (test session) |
| [`components/tpup/core/`](components/tpup/core/) | `Common`: shared math types and robot constants |
| [`components/tpup/hardware/`](components/tpup/hardware/) | `Board` (I2C setup, pins, addresses), `Servo`, `IMU` |
| [`components/tpup/kinematics/`](components/tpup/kinematics/) | `IK`, `Spot_Model`, `Transformer` |
| [`components/tpup/gait/`](components/tpup/gait/) | `Gait_Controller`, `TrapezoidPath` |
| [`components/tpup/estimation/`](components/tpup/estimation/) | `Filter`, `State_Solver` (planned) |
| [`components/tpup_test/`](components/tpup_test/) | `Test_Session`, `Self_Test` |
| [`sim/`](sim/) | Host simulation build and shim headers |
| [`docs/`](docs/) | IK audit and hardware test logs |

Each folder under `components/tpup/` only depends on the folders above it in this table.

Data flow: `GaitController → Transformer → IK → SpotModel → Servos`, with `IMU → Filter` feeding orientation back into `SpotModel`.

## Build & run

The build uses PlatformIO and has two environments. Both compile from the same tree, and a build flag picks the entry point:

```sh
pio run -e walk -t upload -t monitor   # gait control loop (main/walk_main.cpp)
pio run -e test -t upload -t monitor   # interactive hardware test session (main/test_main.cpp)
```

VS Code tasks for both environments are in `.vscode/tasks.json` (Ctrl+Shift+B).

### Host simulation

`sim/` contains shim headers that let the control loop compile and run on a PC, so you can inspect the gait and kinematics math without hardware:

```powershell
.\sim\build_sim.ps1 -Run
```

## Testing

- **Self tests:** pure-math checks of IK, Transformer and TrapezoidPath that run on boot.
- **Hardware test session:** an interactive serial walkthrough covering the IMU (configuration, saturation, calibration, six-face orientation) and the servos (sweeps, limit saturation, protractor-verified angles). Results are saved to NVS across reboots and logged in [`docs/test_results.md`](docs/test_results.md).

## Status

- [x] IK, transforms and trapezoid gait verified
- [x] Servo calibration and limit protection
- [x] IMU validated
- [x] Open-loop walking
- [ ] IMU feedback in the control loop
- [ ] State estimation (`State_Solver`)
