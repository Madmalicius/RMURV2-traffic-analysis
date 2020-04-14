import cv2
import numpy as np


# Load images
stillImg = cv2.imread("still.jpg")
stillImgPaint = cv2.imread("still_mask.jpg")
googleImg = cv2.imread("google.png")
googleImgPaint = cv2.imread("google_mask.png")

googleCoords = np.array([[185,308],[774,327],[557,575],[151,479]], dtype="float32") # Order of points: grey house, red house, red shag, grey shag
stillCoords = np.array([[464,467],[566,196],[823,280],[752,543]], dtype="float32")# Order of points: grey house, red house, red shag, grey shag

M = cv2.getPerspectiveTransform(stillCoords, googleCoords)

warped = cv2.warpPerspective(stillImg, M, (1280, 720))
cv2.imshow("Warped", warped)
cv2.imwrite("warpedStill.jpg", warped)

cv2.waitKey(0)
