from GUI import GUI
from HAL import HAL
import numpy as np
import cv2


def detect_red(img):
    # define the list of boundaries
    redBajo1 = np.array([0, 100, 20], np.uint8)
    redAlto1 = np.array([8, 255, 255], np.uint8)
    redBajo2=np.array([175, 100, 20], np.uint8)
    redAlto2=np.array([179, 255, 255], np.uint8)
    
    # create NumPy arrays from the boundaries
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRed = cv2.add(maskRed1, maskRed2)
    # find the colors within the specified boundaries and apply
    # the mask
    # mask = cv2.inRange(img, lower, upper)
    output = cv2.bitwise_and(img, img, mask = maskRed)
    #result = img.copy()
    contours, hierarchy = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours = (img, contours, -1, (0, 255, 0), 1)
    
    m = cv2.moments(contours)
    
    x = m['m10']/m['m00']
    y = m['m01']/m['m00']
    print('x=',x, 'y=',y)
     
    cv2.circle(image, (int(x),int(y)), 5, 255, 5)
    
    return output


while True:
    image = HAL.getImage()
    output = detect_red(image)
    

    # show the images
    GUI.showImage(image)
    