from picamera2 import Picamera2
from datetime import datetime
from threading import Thread, Event
import time
import numpy as np


class CaptureThread(Thread):
    """
    A thread class to capture an image using a Picamera2 instance.
    """

    def __init__(self, camera, event, timestamp):
        super().__init__()
        self.camera = camera
        self.timestamp = timestamp
        self.image = None
        self.event = event

    def run(self):
        """
        Captures an image using the camera and stores it as a numpy array.
        """
        self.camera.start()
        self.event.wait()  # Wait for the signal to capture
        self.image = self.camera.capture_array()  # Capture the image as a numpy array
        self.camera.stop()
        print(f"Image captured with timestamp {self.timestamp}")


def capture_stereo_images():
    """
    Captures stereo images from two cameras simultaneously and returns them as numpy arrays.
    :return: Tuple of (left_image, right_image), where each is a numpy array.
    """
    # Initialize both cameras
    cam1 = Picamera2(camera_num=1)  # Camera connected to port 0
    cam2 = Picamera2(camera_num=0)  # Camera connected to port 1
    resolution = (3280, 2464)

    # Configure both cameras with the desired resolution for still capture
    cam1.configure(cam1.create_still_configuration(main={"size": resolution, "format": "RGB888"}))
    cam2.configure(cam2.create_still_configuration(main={"size": resolution, "format": "RGB888"}))

    # Set camera controls including exposure, gain, and tungsten white balance
    camera_controls = {
        "ExposureTime": 5000,        # Your existing exposure time
        "AnalogueGain": 8.0,          # Your existing gain
        "AeEnable": False,            # Disable auto-exposure
        "AwbEnable": True,            # Disable auto white balance
    }

    cam1.set_controls(camera_controls)
    cam2.set_controls(camera_controls)

    # Record the timestamp before capturing the images
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S.%f")  # Include microseconds

    # Create an event to synchronize the threads
    capture_event = Event()

    # Create threads to capture images with the same timestamp
    thread1 = CaptureThread(cam1, capture_event, f"{timestamp}_left")
    thread2 = CaptureThread(cam2, capture_event, f"{timestamp}_right")

    # Start both threads
    thread1.start()
    thread2.start()

    # Trigger both cameras to capture at the same time
    capture_event.set()

    # Wait for both threads to complete
    thread1.join()
    thread2.join()

    # Retrieve the images from the threads
    left_image = thread1.image
    right_image = thread2.image

    print(f"Stereo images captured at {timestamp}.")
    return left_image, right_image
