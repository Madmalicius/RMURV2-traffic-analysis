import cv2
import numpy as np
from math import sin, cos, sqrt, atan2, radians

def perspective_transform(image):
    """Warps image to birdseye perspective

    Arguments:
        image {image} -- image or frame to be warped

    """

    googleCoords = np.array([[185,308],[774,327],[557,575],[151,479]], dtype="float32") # Order of points: grey house, red house, red shag, grey shag
    stillCoords = np.array([[464,467],[566,196],[823,280],[752,543]], dtype="float32")# Order of points: grey house, red house, red shag, grey shag

    M = cv2.getPerspectiveTransform(stillCoords, googleCoords)

    warped = cv2.warpPerspective(image, M, (1280, 720))

    return warped

def velocity(currCoords, prevCoords):

    R = 6373.0 #Approx radius of the earth in km
    frameRate = 20

    latGreyShag = radians(55.381893)
    lonGreyShag = radians(10.363923)
    latRedShag = radians(55.381617)
    lonRedShag = radians(10.365979)

    pointGreyShag = [151,479]
    pointRedShag = [557,575]

    dlon = lonRedShag - lonGreyShag
    dlat = latRedShag - latGreyShag

    a = sin(dlat / 2)**2 + cos(latGreyShag) * cos(latRedShag) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    meterDist = R * c * 10**3

    pixelDist = sqrt( ((pointRedShag[0]-pointGreyShag[0])**2)+((pointRedShag[1]-pointGreyShag[1])**2) )

    #print("meter Distance:",meterDist)
    #print("pixelDist:",pixelDist)

    meterPrPixel = meterDist / pixelDist
    difPixel = sqrt((currCoords[0]-prevCoords[0])**2+(currCoords[1]-prevCoords[1])**2)

    vel = (difPixel * meterPrPixel) * frameRate * 3.6

    return vel