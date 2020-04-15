#!/usr/bin/env python

import rosbag
from std_msgs.msg import Int32, String
import cv2
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
from Exercise4 import perspective_transform

#Load files
#cap = cv2.VideoCapture('../traffic_analysis_from_drones/data/traffic_video_dyrskuepladsen.mp4')
paintedfile = "still_painted.jpg"
paint_img= cv2.imread(paintedfile)
#Initialize background subtractor
backSub = cv2.createBackgroundSubtractorMOG2(history=10, varThreshold=20, detectShadows=False)


#Create mask for only road detection
mask = cv2.inRange(perspective_transform(paint_img), (0, 0, 240 ), (40, 40, 255))

#Used for still image saving
#init = 1

bag=rosbag.Bag('../../Stableframe.bag')
bridge=CvBridge()
"""
#Values for roi
y_init = 450
x_init = 520
w_init = 595-520
h_init = 656-470
"""
y_init = 160
x_init = 375
h_init = 445-160
w_init = 528-375
# Columns = max tracked cars
cars = np.zeros((4,2))
check = np.zeros((4,1))

# Counter for detecting if a car is outside image
counter = 0

for topic, msg, t in bag.read_messages(topics=['stabilized_frame']): # The topic is found by the cmd-line " rosbag info Stableframe.bag"
    '''
    #Print still image to use for mask
    if init == 1:
        stillframe=bridge.imgmsg_to_cv2(msg,"bgr8")
        cv2.imwrite("still.jpg", stillframe)
        init = 0
    '''
    frame = bridge.imgmsg_to_cv2(msg,"bgr8")

    frame = perspective_transform(frame)

    #ret, frame = cap.read()
    stream = cv2.bitwise_and(frame, frame, mask=mask)
    
    #Apply background subtraction
    fgMask = backSub.apply(stream)
    
    #Apply roi
    roi = fgMask[x_init:x_init+w_init, y_init:y_init+h_init]
    roi_frame = frame[x_init:x_init+w_init, y_init:y_init+h_init]
    
    #Detect contours
    _, contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    #Draw squares around contours and save the coordinates in an array
    for contour in contours:
        (x, y, w, h) = cv2.boundingRect(contour)

        if cv2.contourArea(contour) < 20:
            continue
        cv2.rectangle(roi_frame, (x,y), (x+w, y+h), (0, 255, 0), 2)
        
        #Check if the car is already tracked, if so update. Else give spot on array
        for k in range(len(cars)):
            if cars[k, 0]-x <= 15 and cars[k, 1]-y <= 15:
                cars[k, 0] = x
                cars[k, 1] = y
                check[k] +=1
                break
    #print(cars)    
    
    #Check if car is still in frame for tracking every 5 frames
    counter +=1
    if counter == 5:
        for k in range(len(cars)):
            if check[k] == 0:
                cars[k, 0] = 0
                cars[k, 1] = 0
            check[k] = 0
        counter = 0
    
    cv2.imshow("feed", roi_frame)
    cv2.imshow("mask", roi)

    #Cancel with escape
    if cv2.waitKey(30) == 27:
        break