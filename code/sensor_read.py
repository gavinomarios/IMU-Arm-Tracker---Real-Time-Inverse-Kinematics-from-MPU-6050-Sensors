import smbus
import time

# I2C addresses for the two sensors
ADDR_UPPER = 0x68  # upper arm IMU (AD0 low)
ADDR_FOREARM = 0x69  # forearm IMU (AD0 high)

# MPU-6050 Registers
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B  # starting register for accel readings
GYRO_XOUT_H  = 0x43  # starting register for gyro readings

bus = smbus.SMBus(1)  # Initialize I2C bus (1 for Raspberry Pi/Jetson I2C-1)

def initialize_mpu(addr):
    # Wake up the MPU-6050 (clear sleep bit)
    bus.write_byte_data(addr, PWR_MGMT_1, 0)
    # (Optional) Set accelerometer full-scale to ±2g (0x1C register) and gyro to ±250°/s (0x1B register)
    # bus.write_byte_data(addr, 0x1C, 0x00)  # 0x00 = ±2g
    # bus.write_byte_data(addr, 0x1B, 0x00)  # 0x00 = ±250 deg/s
    time.sleep(0.1)  # short delay for sensor to stabilize

# Initialize both IMUs
initialize_mpu(ADDR_UPPER)
initialize_mpu(ADDR_FOREARM)

def read_raw_values(addr):
    # Read 14 bytes of consecutive data starting from ACCEL_XOUT_H
    data = bus.read_i2c_block_data(addr, ACCEL_XOUT_H, 14)
    # Convert the data to signed 16-bit integers
    accel_x = (data[0] << 8) | data[1]
    accel_y = (data[2] << 8) | data[3]
    accel_z = (data[4] << 8) | data[5]
    temp   = (data[6] << 8) | data[7]            # temperature (not used here)
    gyro_x = (data[8] << 8) | data[9]
    gyro_y = (data[10] << 8) | data[11]
    gyro_z = (data[12] << 8) | data[13]
    # Convert to signed (two's complement)
    def to_signed(n):
        return n - 65536 if n > 32767 else n
    accel_x = to_signed(accel_x);  accel_y = to_signed(accel_y);  accel_z = to_signed(accel_z)
    gyro_x  = to_signed(gyro_x);   gyro_y  = to_signed(gyro_y);   gyro_z  = to_signed(gyro_z)
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

#Calibration Function
def calibrate_mpu(addr, samples=200):
    print(f"Calibrating sensor at address {hex(addr)}...")
    accel_offset = [0, 0, 0]
    gyro_offset = [0, 0, 0]
    
    for _ in range(samples):
        ax, ay, az, gx, gy, gz = read_raw_values(addr)
        accel_offset[0] += ax
        accel_offset[1] += ay
        accel_offset[2] += az
        gyro_offset[0] += gx
        gyro_offset[1] += gy
        gyro_offset[2] += gz
        time.sleep(0.01)  # slight delay between samples

    # Average the offsets
    accel_offset = [x / samples for x in accel_offset]
    gyro_offset = [x / samples for x in gyro_offset]
    
    print(f"Calibration complete for {hex(addr)}")
    print(f"Accel Offset: {accel_offset}")
    print(f"Gyro Offset: {gyro_offset}")
    return accel_offset, gyro_offset

# Run calibration once before the loop
accel_offset_upper, gyro_offset_upper = calibrate_mpu(ADDR_UPPER)
accel_offset_forearm, gyro_offset_forearm = calibrate_mpu(ADDR_FOREARM)


# Replace these offsets from the calibration: 
accel_offset_upper = [0, 0, 0]   # e.g., small offsets if sensor not perfectly level
gyro_offset_upper  = [0, 0, 0]   # gyro biases
accel_offset_forearm = [0, 0, 0]
gyro_offset_forearm  = [0, 0, 0]

# Reading loop (30 Hz)
import math
dt = 1/30.0  # time per frame ~0.0333s
while True:
    # Upper arm IMU
    ax_u, ay_u, az_u, gx_u, gy_u, gz_u = read_raw_values(ADDR_UPPER)
    # Forearm IMU
    ax_f, ay_f, az_f, gx_f, gy_f, gz_f = read_raw_values(ADDR_FOREARM)
    # Apply offsets
    ax_u -= accel_offset_upper[0]; ay_u -= accel_offset_upper[1]; az_u -= accel_offset_upper[2]
    gx_u -= gyro_offset_upper[0];  gy_u -= gyro_offset_upper[1];  gz_u -= gyro_offset_upper[2]
    ax_f -= accel_offset_forearm[0]; ay_f -= accel_offset_forearm[1]; az_f -= accel_offset_forearm[2]
    gx_f -= gyro_offset_forearm[0];  gy_f -= gyro_offset_forearm[1];  gz_f -= gyro_offset_forearm[2]
    # Scale to physical units
    Ax_u = ax_u / 16384.0  # in g
    Ay_u = ay_u / 16384.0
    Az_u = az_u / 16384.0
    Gx_u = gx_u / 131.0    # in °/s
    Gy_u = gy_u / 131.0
    Gz_u = gz_u / 131.0
    Ax_f = ax_f / 16384.0
    Ay_f = ay_f / 16384.0
    Az_f = az_f / 16384.0
    Gx_f = gx_f / 131.0
    Gy_f = gy_f / 131.0
    Gz_f = gz_f / 131.0

    # ... (pass these values to the filter and kinematics computations below)
    time.sleep(dt)
