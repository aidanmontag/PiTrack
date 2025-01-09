import RPi.GPIO as GPIO
from time import sleep

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for the LEDs
LED1_PIN = 17
LED2_PIN = 27

# Set up the pins as output
GPIO.setup(LED1_PIN, GPIO.OUT)
GPIO.setup(LED2_PIN, GPIO.OUT)

try:
    print("Turning LEDs on...")
    # Turn on the LEDs
    GPIO.output(LED1_PIN, GPIO.HIGH)
    GPIO.output(LED2_PIN, GPIO.HIGH)

    # Wait for 30 seconds
    sleep(60)

    print("Turning LEDs off...")
    # Turn off the LEDs
    GPIO.output(LED1_PIN, GPIO.LOW)
    GPIO.output(LED2_PIN, GPIO.LOW)

except KeyboardInterrupt:
    print("\nScript interrupted. Cleaning up GPIO...")

finally:
    # Clean up GPIO to reset the pins
    GPIO.cleanup()

print("Done.")
