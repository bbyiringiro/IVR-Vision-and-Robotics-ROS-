#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class RobotController3D():

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
        self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
        cam1_sub = message_filters.Subscriber("/camera1/robot/image_raw", Image)
        cam2_sub = message_filters.Subscriber("/camera2/robot/image_raw", Image)
        sync = message_filters.TimeSynchronizer([cam1_sub, cam2_sub], 2)
        sync.registerCallback(self.callback)

        self.bridge = CvBridge()


    def detect_blue(self,imageXZ,imageYZ):
        mask_xz = cv2.inRange(imageXZ, (100, 0, 0), (255, 0, 0))
        mask_yz = cv2.inRange(imageYZ, (100, 0, 0), (255, 0, 0))

        kernel = np.ones((5, 5), np.uint8)
        mask_xz = cv2.dilate(mask_xz, kernel, iterations=3)
        mask_yz = cv2.dilate(mask_yz, kernel, iterations=3)
        M1 = cv2.moments(mask_xz)
        cx = int(M1['m10'] / M1['m00'])
        cz = int(M1['m01'] / M1['m00'])

        M2 = cv2.moments(mask_yz)
        cy = int(M2['m10'] / M2['m00'])
        cz1 = int(M2['m01'] / M2['m00'])
        cz = (cz+cz1)/2 # take average of Z returned from the two camera


        return np.array([cx, cy, cz])

    def detect_yellow(self,imageXZ,imageYZ):
        mask_xz = cv2.inRange(imageXZ, (0, 100, 100), (0, 255, 255))
        mask_yz = cv2.inRange(imageYZ, (0, 100, 100), (0, 255, 255))

        kernel = np.ones((5, 5), np.uint8)
        mask_xz = cv2.dilate(mask_xz, kernel, iterations=3)
        mask_yz = cv2.dilate(mask_yz, kernel, iterations=3)
        M1 = cv2.moments(mask_xz)
        cx = int(M1['m10'] / M1['m00'])
        cz = int(M1['m01'] / M1['m00'])

        M2 = cv2.moments(mask_yz)
        cy = int(M2['m10'] / M2['m00'])
        cz1 = int(M2['m01'] / M2['m00'])
        cz = (cz+cz1)/2 # take average of Z returned from the two camera

        return np.array([cx, cy, cz])

    def detect_red(self,imageXZ,imageYZ):
        mask_xz = cv2.inRange(imageXZ, (0, 0, 100), (0, 0, 255))
        mask_yz = cv2.inRange(imageYZ, (0, 0, 100), (0, 0, 255))

        kernel = np.ones((5, 5), np.uint8)
        mask_xz = cv2.dilate(mask_xz, kernel, iterations=3)
        mask_yz = cv2.dilate(mask_yz, kernel, iterations=3)
        M1 = cv2.moments(mask_xz)
        cx = int(M1['m10'] / M1['m00'])
        cz = int(M1['m01'] / M1['m00'])

        M2 = cv2.moments(mask_yz)
        cy = int(M2['m10'] / M2['m00'])
        cz1 = int(M2['m01'] / M2['m00'])
        cz = (cz+cz1)/2 # take average of Z returned from the two camera

        return np.array([cx, cy, cz])

    def detect_green(self,imageXZ,imageYZ):
        mask_xz = cv2.inRange(imageXZ, (0, 100, 0), (0, 255, 0))
        mask_yz = cv2.inRange(imageYZ, (0, 100, 0), (0, 255, 0))

        kernel = np.ones((5, 5), np.uint8)
        mask_xz = cv2.dilate(mask_xz, kernel, iterations=3)
        mask_yz = cv2.dilate(mask_yz, kernel, iterations=3)
        M1 = cv2.moments(mask_xz)
        cx = int(M1['m10'] / M1['m00'])
        cz = int(M1['m01'] / M1['m00'])

        M2 = cv2.moments(mask_yz)
        cy = int(M2['m10'] / M2['m00'])
        cz1 = int(M2['m01'] / M2['m00'])
        cz = (cz+cz1)/2 # take average of Z returned from the two camera

        return np.array([cx, cy, cz])

    def detect_target(self, imageXY, imageYZ):
        pass

    # detect robot end-effector from the image
    def detect_end_effector(self,imageXY, imageYZ):
        pass

    def detect_joint_angles(self, imageXY, imageYZ):
        pass
    def forward_kinematics(self, joint_angles):
        pass

    # Calculate the robot Jacobian
    def calculate_jacobian(self,imageXY, imageYZ):
        pass

    def control_closed(self,imageXY, imageYZ):
        pass

    def callback(self, cam1_data,cam2_data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(cam1_data, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(cam2_data, "bgr8")
        except CvBridgeError as e:
            print(e)
        corr = self.detect_red(self.cv_image1, self.cv_image2)
        print ("red: x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))

        corr = self.detect_green(self.cv_image1, self.cv_image2)
        print ("green:x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))

        corr = self.detect_blue(self.cv_image1, self.cv_image2)
        print ("blue:x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))

        corr = self.detect_yellow(self.cv_image1, self.cv_image2)
        print ("yellow:x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))



        # im1=cv2.imshow('window1', self.cv_image1)
        # im2=cv2.imshow('window2', self.cv_image2)
        # cv2.waitKey(1)

    

    
# call the class
def main(args):
  ic = RobotController3D()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


