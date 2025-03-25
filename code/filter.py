roll_gyro += Gy * dt  
pitch_gyro += Gx * dt  
yaw_gyro += Gz * dt  

# Initialize orientation estimates (radians)
roll_u, pitch_u, yaw_u = 0.0, 0.0, 0.0      # for upper arm IMU
roll_f, pitch_f, yaw_f = 0.0, 0.0, 0.0      # for forearm IMU

alpha = 0.02  # accelerometer weight in complementary filter (tweak as needed)
# In our formula we'll use (1-alpha) for gyro portion and alpha for accel.

# Inside the loop, after getting Ax,Ay,Az and Gx,Gy,Gz for each IMU:
# Compute accel angles (in radians)
pitch_acc_u = math.atan2(Ax_u, math.sqrt(Ay_u**2 + Az_u**2))
roll_acc_u  = math.atan2(Ay_u, math.sqrt(Ax_u**2 + Az_u**2))
pitch_acc_f = math.atan2(Ax_f, math.sqrt(Ay_f**2 + Az_f**2))
roll_acc_f  = math.atan2(Ay_f, math.sqrt(Ax_f**2 + Az_f**2))
# (We ignore yaw_acc since it's not available)

# Integrate gyro to get angles
roll_u_gyro  = roll_u + Gy_u * dt * math.pi/180.0    # gyro was in °/s, convert to rad
pitch_u_gyro = pitch_u + Gx_u * dt * math.pi/180.0
yaw_u        = yaw_u + Gz_u * dt * math.pi/180.0     # yaw solely from gyro
roll_f_gyro  = roll_f + Gy_f * dt * math.pi/180.0
pitch_f_gyro = pitch_f + Gx_f * dt * math.pi/180.0
yaw_f        = yaw_f + Gz_f * dt * math.pi/180.0

# Complementary filter blending
roll_u  = (1 - alpha) * roll_u_gyro  + alpha * roll_acc_u
pitch_u = (1 - alpha) * pitch_u_gyro + alpha * pitch_acc_u
roll_f  = (1 - alpha) * roll_f_gyro  + alpha * roll_acc_f
pitch_f = (1 - alpha) * pitch_f_gyro + alpha * pitch_acc_f
# (yaw angles we keep integrating as is, but we could constrain them within -pi to pi)
