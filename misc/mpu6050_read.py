import smbus
import json
import math
import time

class MPU6050:
    def __init__(self, bus=1, address=0x68):
        self.bus = smbus.SMBus(bus)
        self.address = address

        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0)

        # Load calibration offsets
        with open("calibration.json", "r") as file:
            calibration_data = json.load(file)

        self.gyro_offset = calibration_data['gyro_offset']
        self.accel_offset = calibration_data['accel_offset']

        self.last_time = time.time()
        self.gyro_angle = {'x': 0, 'y': 0, 'z': 0}

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) | low

        if value > 32767:
            value = value - 65536
        return value

    def get_calibrated_data(self):
        gyro_x = self.read_raw_data(0x43) / 65.5 - self.gyro_offset['x']
        gyro_y = self.read_raw_data(0x45) / 65.5 - self.gyro_offset['y']
        gyro_z = self.read_raw_data(0x47) / 65.5 - self.gyro_offset['z']

        accel_x = self.read_raw_data(0x3B) / 4096.0 - self.accel_offset['x']
        accel_y = self.read_raw_data(0x3D) / 4096.0 - self.accel_offset['y']
        accel_z = self.read_raw_data(0x3F) / 4096.0 - self.accel_offset['z']

        return {'gyro': (gyro_x, gyro_y, gyro_z), 'accel': (accel_x, accel_y, accel_z)}

    def get_angles(self):
        calibrated = self.get_calibrated_data()
        gyro_x, gyro_y, gyro_z = calibrated['gyro']
        accel_x, accel_y, accel_z = calibrated['accel']

        # Calculate time difference
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Calculate accelerometer angles
        acc_pitch = math.degrees(math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)))
        acc_roll = math.degrees(math.atan2(-accel_x, accel_z))

        # Complementary filter
        self.gyro_angle['x'] += gyro_x * dt
        self.gyro_angle['y'] += gyro_y * dt
        self.gyro_angle['z'] += gyro_z * dt

        pitch = (0.98 * self.gyro_angle['x'] + 0.02 * acc_pitch)
        yaw = 0.98 * self.gyro_angle['y'] + 0.02 * acc_roll
        roll = self.gyro_angle['z']

        return pitch, roll, yaw

# Usage example
if __name__ == "__main__":
    mpu = MPU6050()
    try:
        while True:
            pitch, roll, yaw = mpu.get_angles()
            print(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}, Yaw: {yaw:.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Program stopped by user.")
