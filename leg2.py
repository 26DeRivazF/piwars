import time
import board
import adafruit_bno055
import ThunderBorg3 as ThunderBorg
import sys
import math
global TB
bias = 1.165

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
KP = 0.0001# Proportional constant
KI = 0.0000 # Integral constant
KD = 0.000 # Derivative constant

# PID variables
prev_error = 0
integral = 0

# Target heading angle
target_heading = 90.0
minvel = 0.4

# Function to calculate the PID control output
def pid_controller(target):
    #proportional part
    prev_error = 0
    total_error = 0
    target = target%360
    """
    while target > 180:
        target -=360
    while target < -180:
        target +=360
        """
    error = target - float(sensor.euler[0])
    if error > 180:
        error = -(360 - error)
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
        try:
            error = target - float(sensor.euler[0])
            if error > 180:
                error = -(360 - error)
            if error <=-180:
                error = error + 360
        except:
            error = prev_error
    TB.MotorsOff()
    return error
# Main loop
def MoveForward(distance,bias):
    constantSpeed = 0.9
    TB.SetMotor1(0.5*bias)
    TB.SetMotor2(-0.5)
    time.sleep(distance/constantSpeed)
    TB.MotorsOff()
def MoveBackward(distance,bias):
    constantSpeed = 0.9
    TB.SetMotor1(-0.5*bias)
    TB.SetMotor2(0.5)
    time.sleep(distance/constantSpeed)
    TB.MotorsOff()
    """
try:
    #error = pid_controller(180)
    #MoveForward(1, bias)
    #time.sleep(0.1)
    for i in range(5):
        pid_controller(90)
        pid_controller(180)
        pid_controller(90)
        pid_controller(0)
    
    #MoveForward(2.4, bias)
    #print("{} {}".format(error, sensor.euler[0]))

except KeyboardInterrupt:
    # Stop motors on keyboard interrupt
    TB.MotorsOff()
    """
