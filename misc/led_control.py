import RPi.GPIO as GPIO
import time

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)  # LED pin
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button pin (with pull-up resistor)

# Set up PWM on pin 5 with an initial frequency of 1000 Hz (1 kHz)
pwm = GPIO.PWM(5, 1000)  
pwm.start(0)  # Start PWM with 0% duty cycle (LED off)

# Function to set LED brightness
def set_brightness(brightness):
    pwm.ChangeDutyCycle(brightness)
    print(f"LED brightness is {brightness}%")

# Button press counter
press_count = 0

# Function to wait for button press
def wait_for_button_press():
    global press_count
    while True:
        input_state = GPIO.input(16)
        if input_state == GPIO.LOW:  # Button is pressed (active-low)
            time.sleep(0.2)  # Debounce delay
            press_count += 1
            if press_count == 1:
                set_brightness(25)  # 25% brightness
            elif press_count == 2:
                set_brightness(50)  # 50% brightness
            elif press_count == 3:
                set_brightness(75)  # 75% brightness
            elif press_count == 4:
                set_brightness(100)  # 100% brightness
            elif press_count == 5:
                set_brightness(0)  # Turn off LED
                press_count = 0  # Reset press count
            while GPIO.input(16) == GPIO.LOW:  # Wait for button release
                time.sleep(0.1)

# Main loop
try:
    print("Press the button to change LED brightness.")
    while True:
        wait_for_button_press()

except KeyboardInterrupt:
    print("Exiting program")
finally:
    pwm.stop()  # Stop PWM before exiting
    GPIO.cleanup()  # Reset GPIO settings before exiting

