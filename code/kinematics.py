import numpy as np

def euler_to_rot_matrix(roll, pitch, yaw):
    # Compute rotation matrix from Euler angles (roll, pitch, yaw)
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    # Rotation matrix components (assuming Z-yaw, Y-pitch, X-roll)
    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [0,   0,  1]])
    Ry = np.array([[cy, 0, sy],
                   [0,  1,  0],
                   [-sy,0, cy]])
    Rx = np.array([[1,  0,   0],
                   [0,  cx, -sx],
                   [0,  sx,  cx]])
    # Combined rotation: R = Rz * Ry * Rx (matrix multiplication)
    R = Rz.dot(Ry).dot(Rx)
    return R

# Compute unit vectors along each segment (assuming IMU Y-axis is along the arm)
R_upper = euler_to_rot_matrix(roll_u, pitch_u, yaw_u)
R_fore  = euler_to_rot_matrix(roll_f, pitch_f, yaw_f)
# Local unit vector along arm (y-axis)
y_local = np.array([0, 1, 0])
u_upper_vec = R_upper.dot(y_local)   # world coord unit vector along upper arm
u_fore_vec  = R_fore.dot(y_local)    # world coord unit vector along forearm
# Ensure they are unit length (should be if rotation matrix is orthonormal)
# Compute elbow flexion angle in radians
dot = np.clip(np.dot(u_upper_vec, u_fore_vec), -1.0, 1.0)  # dot product
elbow_angle_rad = math.acos(dot)
elbow_angle_deg = math.degrees(elbow_angle_rad)

output_data = {
    "shoulder_angles": {
        "yaw": round(yaw_u * 180/math.pi, 2),
        "pitch": round(pitch_u * 180/math.pi, 2),
        "roll": round(roll_u * 180/math.pi, 2)
    },
    "elbow_angle": round(elbow_angle_deg, 2),
    "elbow_position": [ round(val, 3) for val in (L_upper * u_upper_vec) ],
    "wrist_position": [ round(val, 3) for val in wrist_pos ]
}

# {
  "shoulder_angles": { "yaw": 15.0, "pitch": 45.0, "roll": 2.5 },
  "elbow_angle": 87.3,
  "elbow_position": [0.20, 0.00, 0.35],
  "wrist_position": [0.45, 0.05, 0.30]
}

