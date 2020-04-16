#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('traffic_analysis_from_drones')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from Exercise4 import perspective_transform, velocity
#from kalman_class import kalman 

paintedfile = "still_painted.jpg"
paint_img= cv2.imread(paintedfile)

#Initialize background subtractor
backSub = cv2.createBackgroundSubtractorMOG2(history=10, varThreshold=20, detectShadows=False)

#Values for roi
y_init = 160
x_init = 375
h_init = 445-160
w_init = 528-375

class CarTracker():
    def __init__(self):
        pass
    # Columns = max tracked cars
    cars = np.zeros((15,4))
    check = np.zeros((15,1))

    #kfIndex=0 
    #kalmanFilters = []
    
    # Counter for detecting if a car is outside image
    counter = 0


    def analyze_frame(self, frame):
        
        frame = perspective_transform(frame)

        #Apply background subtraction
        fgMask = backSub.apply(frame)
        
        #Apply roi
        roi = fgMask[x_init:x_init+w_init, y_init:y_init+h_init]
        roi_frame = frame[x_init:x_init+w_init, y_init:y_init+h_init]
        
        #Detect contours
        _, contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        #Draw squares around contours and save the coordinates in an array
        for contour in contours:
            (x, y, w, h) = cv2.boundingRect(contour)

            if cv2.contourArea(contour) < 15:
                continue
            cv2.rectangle(roi_frame, (x,y), (x+w, y+h), (0, 255, 0), 2)
            
            #Check if the car is already tracked, if so update. Else give spot on array
            for k in range(len(self.cars)):
                if self.cars[k, 0]-x <= 30 and self.cars[k, 1]-y <= 8:
                    #kF_obj=kalman(x,y,self.kfIndex)
                    self.cars[k, 2] = self.cars[k, 0]
                    self.cars[k, 3] = self.cars[k, 1]
                    self.cars[k, 0] = x
                    self.cars[k, 1] = y
                    #kF_obj.run_filter(x,y)
                    self.check[k] +=1
                    #self.kfIndex+=1
                    #self.kalmanFilters.append(kF_obj)
                    cv2.putText(roi_frame, "ID:"+str(k), (x,y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                    cv2.putText(roi_frame, "v = "+str(velocity([x,y],[self.cars[k,2],self.cars[k,3]])), (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                    break
        #print(cars)    
        
        #Check if car is still in frame for tracking every 5 frames
        self.counter +=1
        if self.counter == 5:
            for k in range(len(self.cars)):
                if self.check[k] == 0:
                    self.cars[k, 0] = 0
                    self.cars[k, 1] = 0
                self.check[k] = 0
            self.counter = 0
        return roi_frame


class car_tracker_node:
    def __init__(self):
        rospy.init_node('car_tracker')
        self.image_pub = rospy.Publisher("tracked_cars", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("stabilized_frame", Image, self.callback)

        self.car_tracker = CarTracker()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = self.analyze_image(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        except Exception as e:
            print(e)
        
    def analyze_image(self, image):
        image = self.car_tracker.analyze_frame(image)
        return image


def main(args):
    ic = car_tracker_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    print("Launching the car tracker")
    main(sys.argv)

