#!/usr/bin/env python
import roslib
roslib.load_manifest('optical_ardrone')
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ConeTracker:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("/image_out",Image,queue_size=1)
    self.points_pub = rospy.Publisher("/points_out",Polygon,queue_size=1)
    self.bridge = CvBridge()
    self.setup_detector()
    self.filtered_mask = None

  def start(self):
    self.image_sub = rospy.Subscriber("ardrone/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      self.image_in = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    self.find_cones()
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image_out, "bgr8"))
    except CvBridgeError as e:
      print(e)
    self.points_pub.publish(self.points_poly)

  def find_cones(self):
    im = self.image_in
    # blur it
    im = cv2.GaussianBlur(im, (3,3), 0)
    # conv to HSV
    imhsv = cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    # filter by colour
    mask = cv2.inRange(imhsv, self.markerMin, self.markerMax)
    # grab the shape for scaling later
    height, width = mask.shape
    # and dilate a bit to fill gaps
    mask = cv2.dilate(mask, None, iterations=4)
    # time filtering
    if self.filtered_mask is not None:
      self.filtered_mask = cv2.addWeighted(mask,self.alpha, self.filtered_mask,self.beta,0.0)
      # Detect blobs.
      keypoints = self.detector.detect(self.filtered_mask)
    else:
      keypoints = self.detector.detect(mask)
      self.filtered_mask = mask
    # check if anything found    
    if keypoints:
      rospy.loginfo("found %d blobs" % len(keypoints))
      if len(keypoints) > self.max_blobs:
        # if more than four blobs, keep N largest
        keypoints.sort(key=(lambda kp: kp.size) )
        keypoints=keypoints[0:(self.max_blobs-1)]
    else:
      rospy.loginfo("no blobs")
    # and then sort by x position
    keypoints.sort( key=(lambda kp: kp.pt[0]) )
    # Draw blue circles around detected blobs and return the marked up image    
    #self.image_out = cv2.drawKeypoints(imblur, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    self.image_out = cv2.drawKeypoints(self.filtered_mask, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # and convert to polygon for output
    pg = Polygon()
    for kp in keypoints:
      pt = Point32()
      pt.x = kp.pt[0]/width
      pt.y = kp.pt[1]/height
      pt.z = kp.size/100.0
      pg.points.append(pt)
    self.points_poly = pg

  def setup_detector(self):

    self.max_blobs = 3

    # color thresholds in HSV
    self.markerMin = ( 50, 50, 100)
    self.markerMax = ( 78, 255, 255)

    # blending filters
    self.alpha = 0.4
    self.beta = 1-self.alpha

    # Set up the SimpleBlobdetector with default parameters.
    self.params = cv2.SimpleBlobDetector_Params()

    # look for bright blobs
    self.params.filterByColor = True
    self.params.blobColor = 255
    
    # Change thresholds
    self.params.minThreshold = 150;
    self.params.maxThreshold = 250;
     
    # Filter by Area.
    self.params.filterByArea = True
    self.params.minArea = 150
    self.params.maxArea = 20000
     
    # Filter by Circularity
    self.params.filterByCircularity = False
    self.params.minCircularity = 0.0
    self.params.maxCircularity = 1.0
     
    # Filter by Convexity
    self.params.filterByConvexity = False
    self.params.minConvexity = 0.5
     
    # Filter by Inertia
    self.params.filterByInertia = False
    self.params.minInertiaRatio = 0.5
    
    # make detector
    self.detector = cv2.SimpleBlobDetector(self.params)

def main():
  ct = ConeTracker()
  ct.start()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
