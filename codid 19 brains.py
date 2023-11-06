import cv2
import numpy as np
import time
from picamera2 import Picamera2, Preview


picam = Picamera2()

config = picam.create_preview_configuration()
picam.configure(config)

picam.start_preview(Preview.QTGL)

picam.start()
time.sleep(4)

picam.capture_file("something.jpg")

picam.close()

image = cv2.imread('something.jpg')
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_range1 = (0, 100, 100) # lower range of red color in HSV
upper_range1= (30,255, 255) # upper range of red color in HSV
lower_range2 = (160, 100,100)
upper_range2 = (180, 255, 255)
mask = cv2.inRange(hsv_image, lower_range1, upper_range1)
mask2= cv2.inRange(hsv_image, lower_range2, upper_range2)
combined_mask = cv2.add(mask, mask2)

# Find contours in the combined red mask
contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Check if any contours were found
if contours:
    # Find the contour with the largest area (the biggest red object)
    largest_contour = max(contours, key=cv2.contourArea)

    # Draw a bounding box around the largest red object
    x, y, w, h = cv2.boundingRect(largest_contour)
    centreX = (2*x+w)/2
    centreY = (2*y+h)/2
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    print(centreX, centreY)
    if centreX <= 213:
        print("First third")
    elif centreX <= 426:
        print("Second third")
    else:
        print("Third third")
    # Display or save the image with the bounding box
    cv2.imshow('Biggest Red Object', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No red objects found in the image.")

#color_image = cv2.bitwise_and(image, image, mask=combined_mask)

#resized_image = cv2.resize(color_image, (640, 480))

# Display the color image
#cv2.imshow('Color Image', resized_image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

#green 42-84
