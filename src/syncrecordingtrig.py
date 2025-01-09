import time
import smbus
import json
import math
import threading
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from datetime import datetime
import os
import cv2

# GPIO setup
BUTTON_PIN = 16
LED_PIN = 5
IR_PIN_1 = 17
IR_PIN_2 = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(IR_PIN_1, GPIO.OUT)
GPIO.setup(IR_PIN_2, GPIO.OUT)

# Events to control threading
capture_event = threading.Event()
stop_event = threading.Event()  # Shared stop signal

# Create a directory for saving the images
output_dir = "../output"
os.makedirs(output_dir, exist_ok=True)

# Shared variables for frame counting
frame_count_lock = threading.Lock()
frame_count = 0  # Total frames captured
start_time = None  # Start time for recording_

class MPU6050:
    def __init__(self, bus=1, address=0x68):
        self.bus = smbus.SMBus(bus)
        self.address = address

        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0)

        # Load calibration offsets
        with open("../calibration/mpu_calibration.json", "r") as file:
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


class TimeTriggerThread(threading.Thread):
    """
    A thread that sends a trigger event every 1/24th of a second.
    """

    def __init__(self, trigger_event, stop_event, timestamped_dir):
        super().__init__()
        self.trigger_event = trigger_event
        self.stop_event = stop_event
        self.timestamped_dir = timestamped_dir
        self.accelerometer_file = os.path.join(self.timestamped_dir, "accelerometer.json")
        self.mpu = MPU6050()

    def run(self):
        global frame_count
        interval = 1 / 24  # Time interval in seconds
        next_trigger_time = time.time() + interval

        while not self.stop_event.is_set():
            current_time = time.time()
            if current_time >= next_trigger_time:
                self.trigger_event.set()
                next_trigger_time += interval

                with frame_count_lock:  # Safely update the global frame count
                    frame_count += 1
                    
                self.trigger_event.clear()
                
                calibrated = self.mpu.get_calibrated_data()
                gyro_x, gyro_y, gyro_z = calibrated['gyro']
                accel_x, accel_y, accel_z = calibrated['accel']
                
                data = {
                    'frame': frame_count,
                    'gyro_x': gyro_x,
                    'gyro_y': gyro_y,
                    'gyro_z': gyro_z,
                    'accel_x': accel_x,
                    'accel_y': accel_y,
                    'accel_z': accel_z,
                }
                with open(self.accelerometer_file, 'a') as f:
                    json.dump(data, f)
                    f.write('\n')

            # Optionally add a small sleep to reduce CPU usage
            time.sleep(0.001)


class CameraCaptureThread(threading.Thread):
    """
    A thread that waits for a trigger event and captures a frame when the event is set.
    """

    def __init__(self, camera, trigger_event, stop_event, output_dir, camera_name):
        super().__init__()
        self.camera = camera
        self.trigger_event = trigger_event
        self.stop_event = stop_event
        self.output_dir = output_dir
        self.camera_name = camera_name

    def run(self):
        global frame_count 
        global total_frames       
        while not self.stop_event.is_set():
            if self.trigger_event.wait(timeout=0.1):  # Timeout ensures periodic checking for stop_event
                #timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                frame = self.camera.capture_array()
                #yuv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
                #y_channel, u_channel, v_channel = cv2.split(yuv_frame)
                filename = os.path.join(
                    self.output_dir, f"{self.camera_name}_{frame_count:04d}.jpg"
                )
                cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 100])

                total_frames += 1
                
                #print('succesful captre', self.camera_name)
                self.trigger_event.clear()

def FindExposure(shutter_speed, camera):

    # Start the camera
    camera.start()

    # Lock the shutter speed and enable auto-exposure for gain adjustment
    camera.set_controls({'AeEnable': True, 'ExposureTime': shutter_speed, 'AwbEnable': True})

    # Capture an image or allow some time for the camera to adjust
    time.sleep(.5)  # Give time for the camera to adjust exposure

    metadata = camera.capture_metadata()

    print(metadata)
    
       # Extract the analogue gain
    auto_gain = metadata.get("AnalogueGain", None)
    auto_shutter = metadata.get("ExposureTime", None)

    if auto_shutter > shutter_speed:
        shutter = shutter_speed
        gain = auto_gain * (auto_shutter / shutter_speed)

    else:
        shutter = auto_shutter
        gain = auto_gain

    camera.stop()

    # Return the locked shutter speed and the calculated gain
    return shutter, gain


def main():
    global total_frames, start_time, frame_count
    frame_count = 0
    total_frames = 0

    # Create directories for output
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    timestamped_dir = os.path.join(output_dir, timestamp)  # Timestamped directory
    left_dir = os.path.join(timestamped_dir, "left")
    right_dir = os.path.join(timestamped_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    # Initialize cameras
    camera1 = Picamera2(camera_num=0)
    camera2 = Picamera2(camera_num=1)

    # Set the resolution to quarter resolution (1640x1232)
    resolution = (1640, 1232)
    camera1.configure(camera1.create_video_configuration(main={"size": resolution, "format": "RGB888"}))
    camera2.configure(camera2.create_video_configuration(main={"size": resolution, "format": "RGB888"}))

    print("Waiting for button press...")
    while GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        time.sleep(0.1)
    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
        time.sleep(0.1)

    GPIO.output(IR_PIN_1, GPIO.HIGH)
    GPIO.output(IR_PIN_2, GPIO.HIGH)
    shutter, gain = FindExposure(10000, camera1)
    GPIO.output(IR_PIN_1, GPIO.LOW)
    GPIO.output(IR_PIN_2, GPIO.LOW)

    print(f"gain set at {gain}")
    print(f"shutter set at {shutter}")

    camera_controls = {
        "ExposureTime": shutter,
        "AnalogueGain": gain,
        "AeEnable": False,
        "AwbEnable": False,
    }
    camera1.set_controls(camera_controls)
    camera2.set_controls(camera_controls)
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.output(IR_PIN_1, GPIO.LOW)
    GPIO.output(IR_PIN_2, GPIO.LOW)

    # Start recording
    GPIO.output(LED_PIN, GPIO.HIGH)
    GPIO.output(IR_PIN_1, GPIO.HIGH)
    GPIO.output(IR_PIN_2, GPIO.HIGH)
    camera1.start()
    camera2.start()
    print('begin recording')

    time.sleep(.5)

    # Start threads
    time_thread = TimeTriggerThread(capture_event, stop_event, timestamped_dir)  # Pass timestamped directory
    capture_thread1 = CameraCaptureThread(camera1, capture_event, stop_event, right_dir, "right")
    capture_thread2 = CameraCaptureThread(camera2, capture_event, stop_event, left_dir, "left")
    capture_thread1.start()
    capture_thread2.start()
    time.sleep(0.5)
    time_thread.start()
    start_time = time.perf_counter()

    # Wait for button release
    while GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        time.sleep(0.1)

    stop_event.set()  # Signal threads to stop
    print("Button released, stopping...")
    end_time = time.perf_counter()
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.output(IR_PIN_1, GPIO.LOW)
    GPIO.output(IR_PIN_2, GPIO.LOW)

    time_thread.join()
    capture_thread1.join()
    capture_thread2.join()

    camera1.stop()
    camera2.stop()
    GPIO.cleanup()

    # Calculate average framerate
    total_time = end_time - start_time
    average_fps = (total_frames / 2) / total_time if total_time > 0 else 0

    print(f"Recording finished.")
    print(f"total pulses: {frame_count}")
    print(f"Total frames recorded: {total_frames}")
    print(f"Total recording time: {total_time:.2f} seconds")
    print(f"Average framerate: {average_fps:.2f} FPS")

if __name__ == "__main__":
    main()