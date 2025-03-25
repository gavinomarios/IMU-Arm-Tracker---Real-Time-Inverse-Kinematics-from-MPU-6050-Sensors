# IMU Arm Tracker - Real-Time Inverse Kinematics from MPU-6050 Sensors

## Overview
This repository contains a Python-based real-time inverse kinematics model using two MPU-6050 IMU sensors mounted on a human upper arm and forearm. It calculates the 3D spatial position and orientation of the arm, computes shoulder and elbow joint angles, and generates output data ready for wireless transmission to control a robotic arm. A live 3D visualization using Matplotlib is also included.

The system runs on **Raspberry Pi** or **NVIDIA Jetson Nano**.

---

## Features
- Real-time (30 FPS) accelerometer & gyroscope data acquisition from 2x MPU-6050
- Sensor fusion with Complementary Filter
- 3D spatial position computation for elbow and wrist
- Joint angle calculation (shoulder: yaw, pitch, roll / elbow flexion)
- JSON-ready output format for wireless transmission
- Optional 3D visualization using Matplotlib

---

## Hardware Requirements
- Raspberry Pi 4 / Jetson Nano
- 2x MPU-6050 IMU sensors (GY-521 modules)
- Jumper wires (I2C connection)
- Velcro straps / enclosures for sensor mounting
- Battery pack (if portable)

**Optional for wireless transmission:**
- Wi-Fi (built-in)
- Bluetooth module / XBee (customize `transmit.py`)

---

## Repository Structure
```
imu-arm-tracker/
├── README.md
├── code/
│   ├── sensor_read.py        # MPU-6050 interfacing
│   ├── filter.py             # Complementary filter
│   ├── kinematics.py         # Position and angle calculations
│   ├── transmit.py           # Data serialization/transmission
│   └── visualize.py          # Real-time 3D plot
├── docs/
│   ├── hardware_setup.md
│   └── algorithm_details.md
└── data/
    └── sample_log.csv
```

---

## Installation
```bash
sudo apt update && sudo apt install python3-smbus python3-numpy python3-matplotlib
pip3 install smbus2
```

Ensure I2C is enabled (`raspi-config` or Jetson-IO).

---

## Running the System
### 1. Sensor Calibration
```bash
python3 code/sensor_read.py
```
- Keep the arm stationary to capture offsets.
- Update offsets in `sensor_read.py` or `filter.py`.

### 2. Start Tracking and Visualization
```bash
python3 code/track_and_visualize.py
```
- View real-time 3D arm model.
- Terminal displays angles and positions.

### 3. Transmit Data (optional)
Edit `transmit.py` with your robot’s IP/Bluetooth config:
```bash
python3 code/track_and_send.py
```

---

## Data Output Format
Example JSON frame:
```json
{
  "shoulder_angles": { "yaw": 15.0, "pitch": 45.0, "roll": 2.5 },
  "elbow_angle": 87.3,
  "elbow_position": [0.20, 0.00, 0.35],
  "wrist_position": [0.45, 0.05, 0.30]
}
```

---

## 3D Visualization Sample
The plot shows live motion of the upper arm and forearm segments in 3D space.

Run:
```bash
python3 code/visualize.py
```

---

## Parameters
- **Upper Arm Length** = 1.2 * **Forearm Length** (set `L_forearm` in `kinematics.py`)
- Sampling rate = 30 Hz (adjustable)

---

## Advanced Filtering (Madgwick / Mahony / Kalman)

### Advanced Filtering Adds:
| Improvement               | Effect                                                    |
|---------------------------|------------------------------------------------------------|
| Drift Correction          | Prevents the robot arm from "twisting" over time           |
| Smooth Angle Estimation   | Eliminates jitter in elbow/shoulder angles                 |
| Yaw Stability             | Stabilizes rotation around vertical (good for turning)     |
| Faster Dynamic Response   | Better handling of fast arm movements                      |
| Quaternions               | Avoids gimbal lock issues when computing 3D rotations      |

| Scenario               | Without Filter           | With Madgwick / Mahony / Kalman    |
|------------------------|--------------------------|------------------------------------|
| Static accuracy        | ~5° to 10° error          | ~1° to 3° error                    |
| Fast motion            | Jitter & overshoot        | Smooth, responsive                 |
| Yaw drift after 2 min  | Noticeable                | Minimal to none                    |
| Long-term use          | Needs re-zero / recalib   | Can run much longer, self-corrects |

---

## Further Steps
-Basic inverse kinematics from IMUs  
-Real-time plotting  
-Data serialization  
-ROS integration  
-Advanced sensor fusion (Madgwick/Kalman)  
-Magnetometer support for yaw correction

---

## License
MIT License

---

## Authors
- Gavin Marios Yang

### Contributions welcome!
Pull requests or issues encouraged for improvements or new features.

