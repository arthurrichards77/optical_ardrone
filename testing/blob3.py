# Standard imports
import cv2
import numpy as np;


# Read image
im = cv2.imread("test.jpg")
# blur it
im = cv2.GaussianBlur(im, (3,3), 0)

# conv to HSV
imhsv = cv2.cvtColor(im,cv2.COLOR_BGR2HSV)

# filter by colour
markerMin = ( 59,  50,  30)
markerMax = ( 72, 255, 255)
mask = cv2.inRange(imhsv, markerMin, markerMax)

# and dilate a bit to fill gaps
mask = cv2.dilate(mask, None, iterations=4)

# Set up the SimpleBlobdetector with default parameters.
params = cv2.SimpleBlobDetector_Params()

# look for bright blobs
params.filterByColor = True
params.blobColor = 255
    
# Change thresholds
params.minThreshold = 0;
params.maxThreshold = 256;
     
# Filter by Area.
params.filterByArea = True
params.minArea = 1000
params.maxArea = 20000
     
# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.0
params.maxCircularity = 1.0
     
# Filter by Convexity
params.filterByConvexity = False
#params.minConvexity = 0.5
     
# Filter by Inertia
params.filterByInertia = False
#params.minInertiaRatio = 0.5
    
detector = cv2.SimpleBlobDetector(params)

# Detect blobs.
keypoints = detector.detect(mask)
print keypoints[0]

if keypoints:
    print "found %d blobs" % len(keypoints)
#    if len(keypoints) > 4:
#        # if more than four blobs, keep the four largest
#        keypoints.sort(key=(lambda s: s.size))
#        keypoints=keypoints[0:3]
else:
    print "no blobs"
 
# Draw green circles around detected blobs
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
mask_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
# open windows with original image, mask, res, and image with keypoints marked
#cv2.imshow('blurred input',im)
cv2.imshow('mask',mask_with_keypoints)
cv2.imshow("keypoints", im_with_keypoints)
cv2.waitKey(-1)

