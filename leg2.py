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
KP = 0.0008# Proportional constant
KI = 0.0000 # Integral constant
KD = 0.000 # Derivative constant

# PID variables
prev_error = 0
integral = 0

# Target heading angle
target_heading = 90.0
minvel = 0.45
# Function to calculate the PID control output
def pid_controller(target, minvel):
    sleep(0.1)
    print(sensor.euler[0])
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
    print(error)
    if error >= 180:
        error = -(360 - error)
    if error <=-180:
                error = error + 360
    while abs(error)>=1:
        if error<0:
            motor1val = -minvel*1.17+(error*KP)-(prev_error*KD)-(total_error*KI)
            motor2val = -minvel*1.17+(error*KP)-(prev_error*KD)-(total_error*KI)
            TB.SetMotor1(motor1val)
            TB.SetMotor2(motor2val)
            print("turning left")
           
        elif error>=0:
            motor1val = minvel+(error*KP)+(prev_error*KD)+(total_error*KI)
            motor2val = minvel+(error*KP)+(prev_error*KD)+(total_error*KI)
            TB.SetMotor1(motor1val)
            TB.SetMotor2(motor2val)
            print("turning right")
            print(motor1val)
        total_error += error
        prev_error = error
        #print("{} {}".format(error, sensor.euler[0]))
        try:
            error = target - float(sensor.euler[0])
            print(sensor.euler[0],error)
            if error >= 180:
                error = -(360 - error)
            if error <=-180:
                error = error + 360
        except:
            error = prev_error
        """
        TB.MotorsOff()
        
        sleep(0.1)
        error = target - float(sensor.euler[0])
        print(error)
        minvel -= 0.01
        """
    TB.MotorsOff()
    print(sensor.euler[0],error)
    return error
# Main loop
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
def moveColour(colour):
    #red, green - , blue - 41
    if colour == "blue":
        extra = 0.55
    elif colour == "red":
        extra = 0.23
    elif colour == "green":
        extra = 0.39
    MoveForward(extra+0.68, bias)
    
def Green(hsv_image):
    lower_range = (40, 50, 50) # lower range of gren color in HSV
    upper_range = (80, 255, 255) # upper range of green color in HSV

    # Create a mask for the specified color range
    mask = cv2.inRange(hsv_image, lower_range, upper_range)

    # Calculate the number of pixels that fall within the specified color range
    color_pixels = cv2.countNonZero(mask)
    # Calculate the number of pixels that are the specified color
    cv2.imshow('Mask', mask)
    return color_pixels

def Blue(hsv_image):
    lower_range = (100, 100, 100) # lower range of blue color in HSV
    upper_range = (140, 255, 255) # upper range of blue color in HSV

    # Create a mask for the specified color range
    mask = cv2.inRange(hsv_image, lower_range, upper_range)

    # Calculate the number of pixels that fall within the specified color range
    color_pixels = cv2.countNonZero(mask)
    cv2.imshow('Mask', mask)
    cv2.destroyAllWindows()
    return color_pixels

def Red(hsv_image):
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([20, 255, 255])

    # Define the lower and upper bounds for the second range of red color (in HSV)
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Create masks for the specified color ranges
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

    # Combine the masks using bitwise OR operation
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Calculate the number of pixels that fall within the specified color range
    color_pixels = cv2.countNonZero(red_mask)
    cv2.imshow('Mask', red_mask)
    cv2.destroyAllWindows()
    return color_pixels

def colourCheck():
    picam = Picamera2()
    config = picam.create_preview_configuration()
    picam.configure(config)
    picam.start()
    picam.capture_file("something.jpg")
    picam.close()
    image = cv2.imread('something.jpg')
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red_pixels = Red(hsv_image)
    blue_pixels = Blue(hsv_image)
    green_pixels = Green(hsv_image)
    
    pixelCount = [red_pixels, blue_pixels, green_pixels]
    print(pixelCount)
    pixelCount.sort()
    if pixelCount[-1] == red_pixels:
        return "red"
    elif pixelCount[-1] == blue_pixels:
        return "blue"
    elif pixelCount[-1] == green_pixels:
        return "green"
def maze():
    colour = colourCheck()
    input()
    pid_controller(280, minvel)
    input()
    MoveForward(1.1, bias)
    input()
    pid_controller(0,minvel)
    input()
    moveColour(colour)
    input()
    colour = colourCheck()
    input()
    pid_controller(90,minvel)
    input()
    MoveForward(0.8, bias)
    input()
    pid_controller(0,minvel)
    input()
    moveColour(colour)
    input()
    pid_controller(270,minvel)
    input()
    MoveForward(0.9,bias)
    input()
    pid_controller(0,minvel)
    input()
    #MoveForward(0.5)
colourCheck()
print(colourCheck())
maze()
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
