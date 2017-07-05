#!/usr/bin/env python
import roslib
roslib.load_manifest('optical_ardrone')
import rospy
from geometry_msgs.msg import Polygon, Twist
from brl_drones.rospid import Rospid

class DroneController:

  def __init__(self):
    rospy.init_node('drone_controller', anonymous=True)
    self.twist_pub = rospy.Publisher("ext_vel",Twist,queue_size=1)
    self.height_pid = Rospid(2.0, 0.0, 0.0, 'control/height', lower_lim=-0.3, upper_lim=0.3)
    self.lateral_pid = Rospid(2.0, 0.0, 0.0, 'control/roll', lower_lim=-0.3, upper_lim=0.3)
    self.longitudinal_pid = Rospid(2.0, 0.0, 0.0, 'control/pitch', lower_lim=-0.3, upper_lim=0.3)
    self.yaw_pid = Rospid(1.0, 0.0, 0.0, 'control/yaw', lower_lim=-0.3, upper_lim=0.3)

  def start(self):
    self.points_poly_sub = rospy.Subscriber("points_out",Polygon,self.callback)

  def callback(self,data):
    # need the time
    time_now = rospy.get_time()
    # default message
    t = Twist()
    # check number of blobs provided
    num_points = len(data.points)
    if num_points>0:
      # calculated weighted centroid in x
      size_sum = sum([p.z for p in data.points])
      centroid_x = sum([p.x*p.z for p in data.points])/size_sum    
      # track centroid with yaw      
      t.angular.z = self.yaw_pid.update(centroid_x, 0.5, time_now)
    # the rest only works if exactly three points found
    if num_points<>3:
      rospy.logwarn("Got %d points - was expecting 3" % num_points)
    else:
      # lateral control to line up middle point with outer two
      t.linear.y = self.lateral_pid.update(data.points[1].x - 0.5*data.points[0].x - 0.5*data.points[2].x, 0.0, time_now)
      # height control to line up middle point with outer two
      t.linear.z = self.height_pid.update(data.points[1].y - 0.5*data.points[0].y - 0.5*data.points[2].y, 0.0, time_now)
      # longitudinal control to keep the back two fixed angle apart
      t.linear.x = self.longitudinal_pid.update(data.points[2].x - data.points[0].x, 0.2, time_now)
    # send command anyway
    self.twist_pub.publish(t)

def main():
  ct = DroneController()
  ct.start()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
