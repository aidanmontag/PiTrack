import RPi.GPIO as GPIO
import time
import sys

# GPIO setup
LED_PIN = 5
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

def blink_led():
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH)  # Turn LED on
        time.sleep(0.5)  # Wait for half a second
        GPIO.output(LED_PIN, GPIO.LOW)   # Turn LED off
        time.sleep(0.5)  # Wait for half a second

# Run the blinking LED function in the background
if __name__ == '__main__':
    try:
        blink_led()
    except KeyboardInterrupt:
        GPIO.output(LED_PIN, GPIO.LOW)
        GPIO.cleanup()  # Clean up GPIO on exit
        sys.exit(0)
