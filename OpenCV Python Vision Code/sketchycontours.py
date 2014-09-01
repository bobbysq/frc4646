import cv2 as cv
import numpy as np
import math
from pynetworktables import *


#NetworkTable.SetIPAddress("10.46.46.2")
NetworkTable.SetIPAddress("127.0.0.1")
NetworkTable.SetClientMode()
NetworkTable.Initialize()

table = NetworkTable.GetTable("SmartDashboard")

#cap = cv.VideoCapture("http://FRC:FRC@10.46.46.11/mjpg/video.mjpg")
cap = cv.VideoCapture(1)
cap.set(cv.cv.CV_CAP_PROP_EXPOSURE, -4)

# Define the kernels used for erosion and dilation
kernelErode = np.ones((9,9), np.uint8)
kernelDilate = np.ones((9,9), np.uint8)

# Define range of blue colors in HSV
lower_blue = np.array([100, 70, 30])
upper_blue = np.array([120, 255, 255])

# Define an array with ordered numbers for Center of Mass Calculation
x_distances = np.arange(0,640, dtype=np.uint32)
y_distances = np.arange(0,480, dtype=np.uint32)

# Flood fill mask
contoursbg = np.zeros((480, 640), np.uint8)


while True:
  
    # Capture a frame
    is_connected, frame = cap.read()
    
    # Discards the first frame of the image stream 
    if not is_connected: continue
    
    # Convert frame BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    
    # Threshold the HSV image to filter out the blues
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    cv.imshow('colors', mask)
    
    # Erode the image to get rid of noise
    erode = cv.erode(mask, kernelErode, iterations=2)
    
    # Dialate the image to return the image to normal
    dilate = cv.dilate(erode, kernelDilate, iterations=2)
    
    # Calculate Contours
    contour, heir = cv.findContours(dilate, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    result = np.zeros((480,640), np.uint8)
    
    contourimg = np.zeros((480, 640), np.uint8)
     
    for cnt in contour:
        cv.drawContours(dilate, [cnt], 0, 255, -1)
        cv.drawContours(contourimg, [cnt], 0, 255, 5)
      
      
    # Mask the original image with the mask to show detected regions
    result = cv.bitwise_and(frame, frame, mask= dilate)
    
    # Defines variables for the parameters of the image shape
    height, width, depth = frame.shape

    # Defines the default values for the maximum blob area and the x & y moments of that blob
    maxArea = 0
    largestContour = []
    cnt_xmomentMax = width / 2
    cnt_ymomentMax = height / 2
    
    for cnt in contour:
        M = cv.moments(cnt)
        # Finds the moment of each contour
        cnt_xmoment = int(M['m10']/M['m00'])
        cnt_ymoment = int(M['m01']/M['m00'])
        newArea = int(cv.contourArea(cnt))
        # If the contour is the largest currently detected, set the max x & y moment variables.
        if (newArea > maxArea):
            maxArea = newArea
            cnt_xmomentMax = cnt_xmoment
            cnt_ymomentMax = cnt_ymoment
            largestContour = cnt
        
    if len(largestContour) == 0:
        continue
        
    approx = cv.approxPolyDP(largestContour, 3, True)    
  #  boundingRect = cv.minAreaRect(approx)
    (x,y), circleRadius = cv.minEnclosingCircle(approx)
    circleCenter = (int(x), int(y))
    circleRadius = int(circleRadius)
    boundCircArea = math.pi * (circleRadius ** 2)
    circleFit = maxArea / boundCircArea
 #   print "%f %f" % (maxArea, boundCircArea)
    isCircle = (circleFit > 0.70)
    
    
#     for x in range(0, largestContour.size):
#         cv.approxPolyDP(largestContour[x],)
    # Draws the center of gravity of the largest contour 
    color = (0,0,255)
    if isCircle:
        color = (0,255,0)
       
    cv.circle(result, (circleCenter), circleRadius, color, 2)   
    cv.circle(result, (cnt_xmomentMax, cnt_ymomentMax), 10, color, -1)
    # Sends the X & Y moment variables over NetworkTables
    table.PutNumber('XMoment', cnt_xmomentMax)
    table.PutNumber('YMoment', cnt_ymomentMax)
    
    # Sends the image width variable over NetworkTables
    table.PutNumber('IMAGE_WIDTH', width)
    table.PutNumber('CIRCLE_DIAMETER', circleRadius*2)
    table.PutNumber('MAX_DIAMETER', 335)
    table.PutNumber('MIN_DIAMETER', 90)
    table.PutBoolean('ISCIRCLE', isCircle)
    '''
    # Show a marker at the Center of mass
    if (blob_xpos != 0) and (blob_ypos != 0):
      cv.circle(result, (blob_xpos, blob_ypos), 10, (0,255,0), -1)
    '''
    
    #greyImg = cv.cvtColor(mask, cv.COLOR_HSV2BGR_FULL)
    #greyImg = cv.cvtColor(greyImg, cv.COLOR_BGR2GRAY)
    #circles = cv.HoughCircles (dilate, cv.cv.CV_HOUGH_GRADIENT, 1, dilate.rows/8, 200, 100, 0, 0)
   
    # Finds circles in the contours
    circles = cv.HoughCircles(contourimg, cv.cv.CV_HOUGH_GRADIENT, 1, 20, param1=50,param2=30,minRadius=0,maxRadius=0)

    massInCircle = False
    
    # If any circles are found:
#     if circles is not None:
#         circles = np.uint16(np.around(circles))
#         for i in circles[0,:]:
#             # Determines if the center of gravity of the contour is within a circle
#             deltaX = i[0] - cnt_xmomentMax
#             deltaY = i[1] - cnt_ymomentMax
#             centerDistance = math.sqrt ((deltaX * deltaX) + (deltaY * deltaY))
#             if centerDistance < i[2]:
#                 massInCircle = True
#             # draw the outer circle
#             cv.circle(result,(i[0],i[1]),i[2],(0,255,0),2)
#             # draw the center of the circle
#             cv.circle(result,(i[0],i[1]),2,(0,0,255),3)  

    cv.imshow('contours', contourimg)
    cv.imshow('image', frame)
    #cv.imshow('mask', mask)
    cv.imshow('result', result)
    #cv.imshow('erode', erode)
    cv.imshow('dilate', dilate)
  
    # Break if a key is pressed
    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break

# Closes all windows    
cap.release()
cv.destroyAllWindows()

# BLUES
# Kitchen Table Nighttime settings (Need to divide H by 2 from GIMP
# Dark HSV:  231, 100, 20
# Light HSV: 213, 69,  62

# Kitchen Talbe Sunny morning settings
# Dark  HSV: 231, 64, 56
# Light HSV: 201, 73, 100


# REDS
# Kitchen Talbe Sunny morning settings
# Dark  HSV: 349, 89, 97
# Light HSV: 12, 59, 100