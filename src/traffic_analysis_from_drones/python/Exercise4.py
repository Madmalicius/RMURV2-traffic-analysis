import cv2
import numpy as np

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