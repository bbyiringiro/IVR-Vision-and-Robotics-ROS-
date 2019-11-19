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
        lower_orange = np.array([10, 40, 90],np.uint8)
        upper_orange = np.array([27, 255, 255],np.uint8)
        imageXY = cv2.cvtColor(imageXY,cv2.COLOR_BGR2HSV)
        imageYZ = cv2.cvtColor(imageYZ,cv2.COLOR_BGR2HSV)

        mask_xy = cv2.inRange(imageXY, lower_orange, upper_orange)
        mask_yz = cv2.inRange(imageYZ, lower_orange, upper_orange)


        contours_xy, _ = cv2.findContours(mask_xy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_yz, _ = cv2.findContours(mask_yz, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        try:
            if (len(contours_xy) == 2):
                sphere = contours_xy[1]
            else:
                sphere = contours_xy[0]
            if(cv2.contourArea(sphere)==0):
                M=sphere[0][0]
                cX = M[0]
                cZ_1 = M[1]
            else:
                M = cv2.moments(sphere)
                cX = int(M["m10"] / M["m00"])
                cZ_1 = int(M["m01"] / M["m00"])     
        except:
            print(len(contours_xy))
            cX = 0
            cZ_1 = 0

            

        try:
            if (len(contours_yz) == 2):
                sphere = contours_yz[1]
            else:
                sphere = contours_yz[0]
            if(cv2.contourArea(sphere)==0):
                M=sphere[0][0]
                cY = M[0]
                cZ_2 = M[1]
            else:
                M = cv2.moments(sphere)
                cY = int(M["m10"] / M["m00"])
                cZ_2 = int(M["m01"] / M["m00"]) 
        except Exception as err:
            print(len(contours_yz))
            print(err)
            print (M["m10"])
            cY=0
            cZ_2=0
        cZ=0
        if(cZ_1 !=0 and cZ_2):
            cZ = (cZ_1 + cZ_2)/2
        elif (cZ_1 !=0):
            cZ = cZ_1
        else:
            cZ = cZ_2


        return np.array([cX, cY, cZ])


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
        # corr = self.detect_red(self.cv_image1, self.cv_image2)
        # print ("red: x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))

        # corr = self.detect_green(self.cv_image1, self.cv_image2)
        # print ("green:x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))

        # corr = self.detect_blue(self.cv_image1, self.cv_image2)
        # print ("blue:x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))

        # corr = self.detect_yellow(self.cv_image1, self.cv_image2)
        # print ("yellow:x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))

        print(self.detect_target(self.cv_image1, self.cv_image2))

        im1=cv2.imshow('window1', self.cv_image1)
        im2=cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)

    

    
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


