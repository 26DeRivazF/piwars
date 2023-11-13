import time
import board

import adafruit_bno055

import ThunderBorg3 as ThunderBorg

import sys

import math
 
global TB

TB = ThunderBorg.ThunderBorg()  

TB.Init()

i2c = board.I2C()  # uses board.SCL and board.SDA

# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

sensor = adafruit_bno055.BNO055_I2C(i2c)
 
# If you are going to use UART uncomment these lines

# uart = board.UART()

# sensor = adafruit_bno055.BNO055_UART(uart)
 
last_val = 0xFFFF
 
 
def temperature():

    global last_val  # pylint: disable=global-statement

    result = sensor.temperature

    if abs(result - last_val) == 128:

        result = sensor.temperature

        if abs(result - last_val) == 128:

            return 0b00111111 & result

    last_val = result

    return result
 
def turnLeft(degrees):

    initial = sensor.euler[0]
    if initial - degrees > 0:

        final = initial - degrees

    else:

        final = 360+(initial-degrees)

    print(final)

    print(sensor.euler[0])

    while sensor.euler[0] < final-2 or sensor.euler[0] > final+2:

        TB.SetMotor1(-0.5)

        TB.SetMotor2(-0.5)

        print(sensor.euler[0])

    TB.MotorsOff()

"""

while True:

    print("Accelerometer (m/s^2): {}".format(sensor.acceleration))

    print("Gyroscope (rad/sec): {}".format(sensor.gyro))

    print("Euler angle: {}".format(sensor.euler[0]))

    print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))

    print()

    TB.SetMotor1(0.5)

    TB.SetMotor2(0.5)

    TB.MotorsOff()

    time.sleep(3)

    TB.MotorsOff()

    break

    TB.MotorsOff()

"""

# Initialize BNO055 sensor

# PID constants
KP = 1  # Proportional constant
KI = 0.0  # Integral constant
KD = 0.0  # Derivative constant

# PID variables
prev_error = 0
integral = 0

# Target heading angle
target_heading = 90.0

# Function to calculate the PID control output
def pid_controller(current_heading):
    global prev_error, integral

    # Calculate the error
    error = target_heading - current_heading

    # Update the integral term
    integral += error

    # Calculate the PID output
    output = KP * error + KI * integral + KD * (error - prev_error)

    # Update previous error for the next iteration
    prev_error = error
    output /= 360
    if output < 0:
        output = -0.5 - output
    else:
        output = 0.5 + output
    
    return output

# Main loop
try:
    while True:
        # Read the current heading from BNO055 sensor
        heading = sensor.euler[0]

        # Calculate PID output
        control_output = pid_controller(heading)
        print(control_output)
        # Apply the control output to ThunderBorg motors
        TB.SetMotor1(control_output)
        TB.SetMotor2(control_output)

        # Delay for a short time
        time.sleep(0.1)

except KeyboardInterrupt:
    # Stop motors on keyboard interrupt
    TB.MotorsOff()