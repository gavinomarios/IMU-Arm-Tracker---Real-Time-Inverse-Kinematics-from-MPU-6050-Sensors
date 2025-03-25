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

## ROS Integration

**ROS acts as a communication framework** that connects the IMU-based arm tracker to robots, simulations, and visualizations.

---

- **IMU arm tracker into a ROS Node**
  - It **publishes**:
    - **Joint angles** (`/joint_states`)
    - **3D wrist/hand position** (`/arm_position`)

- **Broadcasted that data in real-time**
  - ➔ Any other ROS Node (robot controller, RViz, Gazebo) can **subscribe** to it.

- **Made your system compatible with ROS tools like**:
  -  **Robot** (real robotic arms)
  -  **RViz** (3D visualization of your arm moving)
  -  **Gazebo** (simulate physics-based robots mirroring your movement)

- **Standardized data using ROS messages like**:
  - `sensor_msgs/JointState`
  - `geometry_msgs/Point`

---

- **Modular:** You don’t control the robot directly — **stream data** that any robot or visualizer can use.
- **Scalable:** Add more subscribers (extra robots, displays) **without changing the tracker**.
- **Reusable:** Use the same setup for:
  -  Real robot arms
  -  3D simulations (Gazebo)
  - Visualizations (RViz)

---

**TL;DR:** ROS turns your IMU tracker into a **real-time data broadcaster**, making it compatible with **robots, simulations, and visualization tools** — **plug-and-play ready** for any ROS ecosystem.

### ROS Publishing Example (Joint Angles and 3D Position):
- Publish `sensor_msgs/JointState` for angles
- Publish `geometry_msgs/Point` for 3D position

Example ROS Python publisher (run with `rosrun`):
```python
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from kinematics import compute_kinematics  # Wrap your kinematics block in this function

rospy.init_node('imu_arm_publisher')
joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
pos_pub = rospy.Publisher('/arm_position', Point, queue_size=10)
rate = rospy.Rate(30)  # Match IMU sampling

while not rospy.is_shutdown():
    # Get the latest kinematics output
    output_data = compute_kinematics()  # output_data is your full JSON-like dict
    
    # Fill JointState with shoulder angles (yaw, pitch, roll) + elbow flexion
    joint_state = JointState()
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ["shoulder_yaw", "shoulder_pitch", "shoulder_roll", "elbow_flex"]
    joint_state.position = [
        np.radians(output_data['shoulder_angles']['yaw']),   # Convert back to radians if needed
        np.radians(output_data['shoulder_angles']['pitch']),
        np.radians(output_data['shoulder_angles']['roll']),
        np.radians(output_data['elbow_angle'])               # elbow angle in degrees → radians
    ]
    
    # Publish wrist (end-effector) 3D position
    wrist_pos = output_data['wrist_position']
    point = Point(wrist_pos[0], wrist_pos[1], wrist_pos[2])

    # Publish both messages
    joint_pub.publish(joint_state)
    pos_pub.publish(point)

    rate.sleep()

#Example ROS /joint_states Published:
header:
  stamp: 123456.789
  frame_id: ''
name: ['shoulder_yaw', 'shoulder_pitch', 'shoulder_roll', 'elbow_flex']
position: [0.261, 0.785, 0.044, 1.523]  # radians
```


### RViz / Gazebo Simulation:
- Add a `robot_state_publisher` and URDF model for visualization.
- View live joint movement in RViz.
- Optional: Use Gazebo + ROS Control or direct plugin for simulation.

Example `roslaunch imu_arm_tracker imu_arm_tracker.launch` to start tracking + RViz visualization.

---

## Further Steps
- Real-time plotting  
- Data serialization  
- Advanced sensor fusion (Madgwick/Kalman)  
- Magnetometer support for yaw correction

---

## License
MIT License

---

## Authors
- Gavin Marios Yang

### Contributions welcome!
Pull requests or issues encouraged for improvements or new features.

