import smbus
import json
import time

class MPU6050:
    def __init__(self, bus=1, address=0x68):
        self.bus = smbus.SMBus(bus)
        self.address = address

        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) | low

        if value > 32767:
            value = value - 65536
        return value

    def calibrate(self, num_samples=500):
        print("Calibrating MPU6050... Please keep the sensor stationary.")
        gyro_sum = {'x': 0, 'y': 0, 'z': 0}
        accel_sum = {'x': 0, 'y': 0, 'z': 0}

        for _ in range(num_samples):
            gyro_sum['x'] += self.read_raw_data(0x43) / 65.5  # Gyro x-axis
            gyro_sum['y'] += self.read_raw_data(0x45) / 65.5  # Gyro y-axis
            gyro_sum['z'] += self.read_raw_data(0x47) / 65.5  # Gyro z-axis

            accel_sum['x'] += self.read_raw_data(0x3B) / 4096.0  # Accel x-axis
            accel_sum['y'] += self.read_raw_data(0x3D) / 4096.0  # Accel y-axis
            accel_sum['z'] += self.read_raw_data(0x3F) / 4096.0  # Accel z-axis

            time.sleep(0.01)

        # Average offsets
        gyro_offset = {
            'x': gyro_sum['x'] / num_samples,
            'y': gyro_sum['y'] / num_samples,
            'z': gyro_sum['z'] / num_samples
        }

        accel_offset = {
            'x': accel_sum['x'] / num_samples,
            'y': accel_sum['y'] / num_samples,
            'z': (accel_sum['z'] / num_samples) - 1  # Gravity correction
        }

        return gyro_offset, accel_offset

# Calibration process
if __name__ == "__main__":
    mpu = MPU6050()
    gyro_offset, accel_offset = mpu.calibrate()

    # Save offsets to file
    calibration_data = {'gyro_offset': gyro_offset, 'accel_offset': accel_offset}
    with open("calibration.json", "w") as file:
        json.dump(calibration_data, file, indent=4)

    print("Calibration complete. Offsets saved to 'calibration.json'.")
