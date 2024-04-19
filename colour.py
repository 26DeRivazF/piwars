import cv2
import numpy as np
from time import sleep
from picamera2 import Picamera2, Preview


picam = Picamera2()

config = picam.create_preview_configuration()
picam.configure(config)

#picam.start_preview(Preview.QTGL)

def Green(picture):
    lower_range = (40, 50, 50) # lower range of gren color in HSV
    upper_range = (80, 255, 255) # upper range of green color in HSV

    # Create a mask for the specified color range
    mask = cv2.inRange(picture, lower_range, upper_range)

    # Calculate the number of pixels that fall within the specified color range
    color_pixels = cv2.countNonZero(mask)
    # Calculate the number of pixels that are the specified color
    cv2.imshow('Mask', mask)
    sleep(1)
    return color_pixels

def Blue(picture):
    lower_range = (100, 100, 100) # lower range of blue color in HSV
    upper_range = (140, 255, 255) # upper range of blue color in HSV

    # Create a mask for the specified color range
    mask = cv2.inRange(picture, lower_range, upper_range)

    # Calculate the number of pixels that fall within the specified color range
    color_pixels = cv2.countNonZero(mask)
    cv2.imshow('Mask', mask)
    sleep(1)
    cv2.destroyAllWindows()
    return color_pixels

def Red(picture):
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([20, 255, 255])

    # Define the lower and upper bounds for the second range of red color (in HSV)
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Create masks for the specified color ranges
    mask1 = cv2.inRange(picture, lower_red1, upper_red1)
    mask2 = cv2.inRange(picture, lower_red2, upper_red2)

    # Combine the masks using bitwise OR operation
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Calculate the number of pixels that fall within the specified color range
    color_pixels = cv2.countNonZero(red_mask)
    cv2.imshow('Mask', red_mask)
    sleep(1)
    cv2.destroyAllWindows()
    return color_pixels
def idColour():
    picam.start()
    picam.capture_file("something.jpg")
    picam.close()
    image = cv2.imread('something.jpg')
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Compare which colour has the most pixels
    red_pixels = Red(hsv_image)
    blue_pixels = Blue(hsv_image)
    green_pixels = Green(hsv_image)

    pixelCount = [red_pixels, blue_pixels, green_pixels]
    print(pixelCount)
    pixelCount.sort()
    if pixelCount[-1] == red_pixels:
        return ("Red")
    elif pixelCount[-1] == blue_pixels:
        return ("Blue")
    elif pixelCount[-1] == green_pixels:
        return ("Green")
print(idColour())