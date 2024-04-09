import RPi.GPIO as GPIO
import time
import math

# Import inverse kinematics functions
from inverse_kinematics import delta_calcInverse

# Define GPIO pins for TB6600 driver connections
STEP_PINS = [17, 18, 19]  # Example GPIO pins for each motor step
DIR_PINS = [27, 28, 29]    # Example GPIO pins for each motor direction

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
for pin in STEP_PINS + DIR_PINS:
    GPIO.setup(pin, GPIO.OUT)

# Define motor movement function
def move_motor(pin, direction, steps):
    GPIO.output(DIR_PINS[pin], direction)
    for _ in range(steps):
        GPIO.output(STEP_PINS[pin], GPIO.HIGH)
        time.sleep(0.001)  # Adjust delay as needed for your motor
        GPIO.output(STEP_PINS[pin], GPIO.LOW)
        time.sleep(0.001)  # Adjust delay as needed for your motor

# Define function to move all motors to specified angles
def move_to_angles(theta1, theta2, theta3):
    # Convert angles to steps or any appropriate calibration
    steps1 = int(theta1 / 360 * 200)  # Example calibration for steps per revolution
    steps2 = int(theta2 / 360 * 200)  # Example calibration for steps per revolution
    steps3 = int(theta3 / 360 * 200)  # Example calibration for steps per revolution
    
    # Move motors
    move_motor(0, GPIO.HIGH, steps1)
    move_motor(1, GPIO.HIGH, steps2)
    move_motor(2, GPIO.HIGH, steps3)

# Main function
if __name__ == "__main__":
    # Desired end-effector position
    x_target = 150  # Example value for x-coordinate
    y_target = 150  # Example value for y-coordinate
    z_target = 150  # Example value for z-coordinate
    
    # Calculate joint angles for the target position
    status, theta1, theta2, theta3 = delta_calcInverse(x_target, y_target, z_target)
    
    if status == 0:
        # Move motors to calculated angles
        move_to_angles(theta1, theta2, theta3)
    else:
        print("Non-existing position")

# Cleanup GPIO
GPIO.cleanup()
