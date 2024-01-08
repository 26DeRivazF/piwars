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
TB.SetMotor1(-0.33)
TB.SetMotor2(-0.33)
time.sleep(5)
TB.MotorsOff()