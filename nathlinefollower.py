
# Import the libraries we need
import UltraBorg
import time

import board
import adafruit_bno055

i2c = board.I2C()  # uses board.SCL and board.SDA

sensor = adafruit_bno055.BNO055_I2C(i2c)

# Start the UltraBorg
UB = UltraBorg.UltraBorg()      # Create a new UltraBorg object
UB.Init()                       # Set the board up (checks the board is connected)

# Loop over the sequence until the user presses CTRL+C
print('Press CTRL+C to finish')
"""
try:
    while True:
        # Read all four ultrasonic values
        usm1 = UB.GetDistance1()
        usm2 = UB.GetDistance2()
        usm3 = UB.GetDistance3()
        usm4 = UB.GetDistance4()
        # Convert to the nearest millimeter
        usm1 = int(usm1)
        usm2 = int(usm2)
        usm3 = int(usm3)
        usm4 = int(usm4)
        # Display the readings
        if usm1 == 0:
            print('#1 No reading')
        else:
            print('#1 % 4d mm' % (usm1))
        if usm2 == 0:
            print('#2 No reading')
        else:
            print('#2 % 4d mm' % (usm2))
        if usm3 == 0:
            print('#3 No reading')
        else:
            print('#3 % 4d mm' % (usm3))
        if usm4 == 0:
            print('#4 No reading')
        else:
            print('#4 % 4d mm' % (usm4))
        print()
        # Wait between readings
        time.sleep(.1)
except KeyboardInterrupt:
    # User has pressed CTRL+C
    print('Done')
"""

import time
import board
import adafruit_bno055
import ThunderBorg3 as ThunderBorg
import sys
import math
global TB
import cv2
import numpy as np
from time import sleep
from picamera2 import Picamera2, Preview


bias = 0.9

TB = ThunderBorg.ThunderBorg()  
TB.Init()
i2c = board.I2C()  # uses board.SCL and board.SDA

# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

sensor = adafruit_bno055.BNO055_I2C(i2c)

#sensor.offsets_magnetometer = (1033, 230, -95)
#sensor.offsets_gyroscope = (-1, -1, -1)

# If you are going to use UART uncomment these lines

# uart = board.UART()

# sensor = adafruit_bno055.BNO055_UART(uart)
 
last_val = 0xFFFF
 

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
KP = 0.1# Proportional constant
KI = 0.0000 # Integral constant
KD = 0.000 # Derivative constant

# PID variables
prev_error = 0
integral = 0

# Target heading angle
target_heading = 90.0
minvel = 0.46
def MoveBackwards(distance,bias):
    bias = 1.165
    constantSpeed = 0.9
    TB.SetMotor1(0.5*bias)
    TB.SetMotor2(-0.5)
    time.sleep(distance/constantSpeed)
    TB.MotorsOff()
def MoveForward(distance,bias):
    constantSpeed = 0.9
    TB.SetMotor1(-0.5*bias)
    TB.SetMotor2(0.5)
    time.sleep(distance/constantSpeed)
    TB.MotorsOff()
    
f = open("errs.txt", "w")
f.close()
def record(value):
    f = open("errs.txt", "a")
    f.write(value + "\n")
    f.close()

# Function to calculate the PID control output

usm1 = UB.GetDistance1()
usm2 = UB.GetDistance2()
usm3 = UB.GetDistance3()
sleep(1)
start_time = time.time()
minvel = 0.3
TB.SetMotor1(-minvel)
TB.SetMotor2(minvel)
prev_error = 0
total_error = 0
while True:
    TB.SetMotor1(-minvel)
    TB.SetMotor2(minvel)
    usm1 = UB.GetDistance1()
    usm2 = UB.GetDistance2()
    usm3 = UB.GetDistance3()
    print(usm2)
    print(usm3)
    error = usm2 - usm3
    record(str(error)+"\n")
    try:
        angle = float(sensor.euler[0])
    except:
        angle = 0
    record("Euler" + str(angle) +"\n")
    while abs(error)>=5 and not(angle > 75 and angle < 285 and (angle - 360 == 0)):
        #dTerm = 0.2*(1-(2.718**-((abs(error)-20)/30)))
        dTerm = 0.3*(abs(error)/200)
        if error < 0:
            dTerm = -1*dTerm
        minvel1 = 0.2
        minvel2 = 0.8
        if error<0:
            motor1val = -minvel1+(dTerm)
            motor2val = minvel1+dTerm # (-dTerm)
            TB.SetMotor1(motor1val)
            TB.SetMotor2(motor2val)
            record("turning left")
           
        elif error>=0:
            motor1val = -minvel1+dTerm #(-dTerm)
            motor2val = minvel1+(dTerm)
            TB.SetMotor1(motor1val)
            TB.SetMotor2(motor2val)
            record("turning right")
        record("M1" + str(motor1val))
        record("M2"+str(motor2val))
        total_error += error
        prev_error = error
        #print("{} {}".format(error, sensor.euler[0]))
        usm1 = UB.GetDistance1()
        usm2 = UB.GetDistance2()
        usm3 = UB.GetDistance3()
        error = usm2-usm3
        try:
            angle = float(sensor.euler[0])
        except:
            angle = 0
        """
        TB.MotorsOff()
        
        sleep(0.1)
        error = target - float(sensor.euler[0])
        print(error)
        minvel -= 0.01
        """
        record(str(error)+"\n")
        record("Euler" + str(sensor.euler) +"\n")
        record("Time %s" % (time.time() - start_time) )
# Main loop





