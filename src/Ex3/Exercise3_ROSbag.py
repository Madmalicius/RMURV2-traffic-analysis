import rosbag
from std_msgs.msg import Int32, String
import cv2
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError

#Load files
#cap = cv2.VideoCapture('../traffic_analysis_from_drones/data/traffic_video_dyrskuepladsen.mp4')
paintedfile = "still_painted1.jpg"
paint_img= cv2.imread(paintedfile)
#Initialize background subtractor
backSub = cv2.createBackgroundSubtractorMOG2(history=10, varThreshold=20, detectShadows=False)
"""
#Save first frame for editing
_, stillframe = cap.read()
cv2.imwrite("still.jpg", stillframe)
"""
#Create mask for only road detection
#mask = cv2.inRange(paint_img, (0, 0, 240 ), (40, 40, 255))  <<<<----- SHould this be used?


bag=rosbag.Bag('../Stableframe.bag')
bridge=CvBridge()
for topic, msg, t in bag.read_messages(topics=['stabilized_frame']): # The topic is found by the cmd-line " rosbag info Stableframe.bag"
    
    frame=bridge.imgmsg_to_cv2(msg,"bgr8")

    #ret, frame = cap.read()
    #stream = cv2.bitwise_and(frame, frame, mask=mask)
    stream = cv2.bitwise_and(frame, frame)
    #Apply background subtraction
    fgMask = backSub.apply(stream)

    #Detect contours
    _, contours, _ = cv2.findContours(fgMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    #Draw squares around contours
    for contour in contours:
        (x, y, w, h) = cv2.boundingRect(contour)

        if cv2.contourArea(contour) < 20:
            continue
        cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 255, 0), 2)

    cv2.rectangle(frame, (10,2), (100,2), (255,255,255), -1)
    #cv2.putText(frame, str(cap.get(cv2.CAP_PROP_POS_FRAMES)), (15, 15),
    #        cv2.FONT_HERSHEY_SIMPLEX, 0.5 , (0,0,0))

    cv2.imshow("feed", frame)
    cv2.imshow("mask", fgMask)


    if cv2.waitKey(30) == 27:
        break