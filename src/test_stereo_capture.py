import numpy as np
import cv2  # OpenCV is used to save images
import RPi.GPIO as GPIO  # Raspberry Pi GPIO library
import matplotlib
matplotlib.use('TkAgg')  # Or 'Qt5Agg' or another interactive backend
import matplotlib.pyplot as plt
from capture_stereo import capture_stereo_images

# Set up GPIO pins
LED_PIN = 5  # LED on GPIO pin 5
BUTTON_PIN = 16  # Button on GPIO pin 16

GPIO.setmode(GPIO.BCM)  # Use BCM GPIO numbering
GPIO.setup(LED_PIN, GPIO.OUT)  # Set LED pin as output
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Set button pin as input with pull-up resistor

def test_capture_stereo_images():
    """
    Test the stereo capture functionality and save the images after waiting for a button press.
    """
    try:
        print("Waiting for button press to start capture...")

        # Wait for the button press (falling edge)
        GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)  # Wait for the button press (falling edge)

        print("Button pressed, starting stereo image capture...")

        # Turn on the LED when the button is pressed
        GPIO.output(LED_PIN, GPIO.HIGH)

        # Capture stereo images
        left_image, right_image = capture_stereo_images()

        # Save the images as PNG files
        left_image_path = "left_image.png"
        right_image_path = "right_image.png"
        cv2.imwrite(left_image_path, left_image)
        cv2.imwrite(right_image_path, right_image)

        # Turn off the LED after saving the images
        GPIO.output(LED_PIN, GPIO.LOW)

        plt.imshow(cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB))
        plt.show()
        plt.imshow(cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB))
        plt.show()

        print(f"Left image saved as {left_image_path}")
        print(f"Right image saved as {right_image_path}")

    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        GPIO.cleanup()  # Clean up GPIO to avoid warnings next time the script runs

if __name__ == "__main__":
    test_capture_stereo_images()





