import socket
import cv2
import os
import RPi.GPIO as GPIO
import time

from datetime import datetime
from picamera2 import Picamera2

import numpy as np
import ArducamDepthCamera as ac

HOST = '0.0.0.0'
PORT = 5005
BUFFER_SIZE = 1024
BASE_DIR = "/home/aidan/Documents/PiTrack/output"

LED_PIN = 5
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

DEPTH_MAX_DISTANCE = 4000
DEPTH_CONFIDENCE_THRESHOLD = 40

def capture_depth(depth_cam, frame_number, session_dir, camera_id):
    # Capture one frame
    frame = depth_cam.requestFrame(2000)
    if frame is not None and isinstance(frame, ac.DepthData):
        depth_buf = frame.depth_data
        #print(depth_buf.max())
        confidence_buf = frame.confidence_data

        # Normalize depth to 16-bit
        result_image = (depth_buf * (65536 / 4000)).astype(np.uint16)
        result_image = np.nan_to_num(result_image)
        print(result_image.max())
        result_image[confidence_buf < DEPTH_CONFIDENCE_THRESHOLD] = (0)

        # Save the image
        filename = f"depth_camera_{camera_id}_{frame_number}.tiff"
        filepath = os.path.join(session_dir, filename)
        cv2.imwrite(filepath, result_image)
        print(f"Captured Depth image: {filepath}")

        depth_cam.releaseFrame(frame)
    else:
        print("Failed to capture a valid frame.")

def create_session_directory(capture_name):
    session_dir = os.path.join(BASE_DIR, capture_name)
    os.makedirs(session_dir, exist_ok=True)
    print(f"Images will be saved in: {session_dir}")
    return session_dir


def capture_image(camera, frame_number, session_dir, camera_id):
    filename = f"camera_{camera_id}_{frame_number}.jpg"
    filepath = os.path.join(session_dir, filename)
    frame = camera.capture_array()
    cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(f"Captured image: {filepath}")


def start_listener(camera, depth_camera, color_dir, depth_dir, camera_id):
    print(f"Listening for UDP pulses on port {PORT}...")
    GPIO.output(LED_PIN, GPIO.HIGH)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('', PORT))
        while True:
            data, addr = s.recvfrom(BUFFER_SIZE)
            message = data.decode().strip()
            if message.upper() == "END":
                print("Received END broadcast. Shutting down listener.")
                GPIO.output(LED_PIN, GPIO.LOW)
                break
            try:
                frame_number = int(message)
                print(f"Pulse received from {addr}, frame {frame_number}")
                capture_image(camera, frame_number, color_dir, camera_id)
                capture_depth(depth_camera, frame_number, depth_dir, camera_id)
            except ValueError:
                print(f"Invalid data from {addr}: {message}")


def initialize_server():
    print(f"Starting camera server on port {PORT}...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print("Waiting for connection...")
        conn, addr = s.accept()
        for i in range(3):
            GPIO.output(LED_PIN, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(LED_PIN, GPIO.LOW)
            time.sleep(0.5)
        with conn:
            print(f"Connected by {addr}")
            data = conn.recv(1024)
            message = data.decode().strip()
            print(f"Received message: {message}")
            # Default values
            camera_id = 'nan'
            capture_name = "DefaultCapture"

            # Parse key-value pairs
            try:
                for pair in message.split(';'):
                    key, value = pair.split('=')
                    if key == "CAMERA_ID":
                        camera_id = value
                    elif key == "CAPTURE_NAME":
                        capture_name = value
            except Exception as e:
                print(f"Failed to parse init message: {e}")

            conn.sendall(b'READY')
            return camera_id, capture_name  


def main():
    camera_id, capture_name = initialize_server()

    session_dir = create_session_directory(capture_name)
    color_dir = os.path.join(session_dir, "color")
    depth_dir = os.path.join(session_dir, "depth")

    os.makedirs(color_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    camera1 = Picamera2(camera_num=0)
    resolution = (1640, 1232)
    camera1.configure(camera1.create_video_configuration(main={"size": resolution}))

    camera_controls = {
        "AeEnable": False,
        "ExposureTime": 20833,
        "AnalogueGain": 5
    }
    camera1.set_controls(camera_controls)

    camera1.start()
    print('Camera 1 started')

    depth_cam = ac.ArducamCamera()
    depth_cam.open(ac.Connection.CSI, 0)
    depth_cam.start(ac.FrameType.DEPTH)
    depth_cam.setControl(ac.Control.RANGE, DEPTH_MAX_DISTANCE)
    depth_cam.getControl(ac.Control.RANGE)
    print('Depth Camera started')

    try:
        start_listener(camera1, depth_cam, color_dir, depth_dir, camera_id)
    finally:
        print("Cleaning up...")
        camera1.stop()
        depth_cam.stop()
        depth_cam.close()
        GPIO.output(LED_PIN, GPIO.LOW)
        GPIO.cleanup()
        print("Shutdown complete.")


if __name__ == '__main__':
    main()


