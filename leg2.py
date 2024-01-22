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
KP = 0.001# Proportional constant
KI = 0 # Integral constant
KD = 0 # Derivative constant

# PID variables
prev_error = 0
integral = 0

# Target heading angle
target_heading = 90.0
minvel = 0.33

# Function to calculate the PID control output
def pid_controller(target):
    #proportional part
    prev_error = 0
    total_error = 0
    error = target - sensor.euler[0]
    while abs(error)>=1:
        if error<0:
            TB.SetMotor1(-minvel+(error*KP)+(prev_error*KD)+(total_error*KI))
            TB.SetMotor2(-minvel+(error*KP)+(prev_error*KD)+(total_error*KI))
        elif error>=0:
            TB.SetMotor1(minvel+(error*KP)+(prev_error*KD)+(total_error*KI))
            TB.SetMotor2(minvel+(error*KP)+(prev_error*KD)+(total_error*KI))
            
        total_error += error
        prev_error = error
        print("{} {}".format(error, sensor.euler[0]))
        error = target - sensor.euler[0]
    TB.MotorsOff()
    return error
# Main loop
try:
    while True:
        error = pid_controller(180)
        time.sleep(0.1)
        print("{} {}".format(error, sensor.euler[0]))

except KeyboardInterrupt:
    # Stop motors on keyboard interrupt
    TB.MotorsOff()
