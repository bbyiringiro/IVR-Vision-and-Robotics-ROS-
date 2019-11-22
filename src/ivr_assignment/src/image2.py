#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import commonLib as lib


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing2', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    self.cam2_data= rospy.Publisher("cam2_data", Float64MultiArray, queue_size=10)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.prev_target_pos =np.array([0.0,8.0], dtype='float64')
    self.prev_yellow_pos = np.array([0.0,0.0], dtype='float64')
    self.prev_blue_pos = np.array([0.0,0.0,2.0], dtype='float64')
    self.prev_green_pos = np.array([0.0,5.0], dtype='float64')
    self.prev_red_pos = np.array([0.0,0.0,7.0], dtype='float64')

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)
    # avoid the craches when the target area is not visible in the camera
    try:
      red_pos = lib.detect_red(self.cv_image2)
      self.prev_red_pos = red_pos
    except:
      red_pos = self.prev_red_pos

    try:
      green_pos = lib.detect_green(self.cv_image2)
      self.prev_green_pos = green_pos
    except:
      green_pos = self.prev_green_pos

    try:
      blue_pos = lib.detect_blue(self.cv_image2)
      self.prev_blue_pos = blue_pos
    except:
      blue_pos = self.prev_blue_pos

    try:
      yellow_pos = lib.detect_yellow(self.cv_image2)
      self.prev_yellow_pos = yellow_pos
    except:
      yellow_pos = self.prev_yellow_pos

    try:
      target_pos = lib.detect_target(self.cv_image2)
      self.prev_target_pos = target_pos
    except:
      target_pos = self.prev_target_pos

    res = red_pos + green_pos + blue_pos + yellow_pos + target_pos

    tmp_res=Float64MultiArray()
    tmp_res.data=res
    if(len(res) !=10):
      print("size should be 10")
      quit(1)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.cam2_data.publish(tmp_res)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


