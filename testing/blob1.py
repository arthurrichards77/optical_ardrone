# Standard imports
import cv2
import numpy as np;

# Read image
im = cv2.imread("test.jpg")
cv2.imshow("Input", im)
cv2.waitKey(-1)

#conv to HSV
imhsv = cv2.cvtColor(im,cv2.COLOR_BGR2HSV)

#region of interest
imrhsv = imhsv[239:319,239:359,:]

#cv2.imshow("Input", imrhsv)
#cv2.waitKey(-1)

mean_hue = np.mean(imrhsv[:,:,0])
hue_tol = 0.2
hue_hi = (1+hue_tol)*mean_hue
hue_lo = (1-hue_tol)*mean_hue
test1 = (imhsv[:,:,0]<=hue_hi)
test2 = (imhsv[:,:,0]>=hue_lo)

mean_sat = np.mean(imrhsv[:,:,1])
sat_tol = 0.3
sat_hi = (1+sat_tol)*mean_sat
sat_lo = (1-sat_tol)*mean_sat
test3 = (imhsv[:,:,1]<=sat_hi)
test4 = (imhsv[:,:,1]>=sat_lo)

print mean_hue, mean_sat

test_all = test1 & test2 & test3 & test4

im_test = (220*test_all).astype('uint8')
cv2.imshow("Output", im_test)
cv2.waitKey(-1)
