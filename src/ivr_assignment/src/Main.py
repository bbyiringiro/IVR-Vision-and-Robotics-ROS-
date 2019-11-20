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
from scipy.optimize import least_squares
import sympy as sym

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
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64') 
        self.error = np.array([0.0,0.0], dtype='float64')  
        self.error_d = np.array([0.0,0.0], dtype='float64') 
        self.joints_ang = np.array([0.0, 0.0, 0.0, 0.0], dtype='float64') 
        self.joints_pos = np.array([[0.0, 0.0, 0.0],
                                    [0.0, 0.0, 2.0],
                                    [0.0, 0.0, 5.0],
                                    [0.0, 0.0, 7.0]], dtype='float64')

        


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
    
    def pixel2meter(self,imageXY, imageYZ):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(imageXY, imageYZ)
      circle2Pos = self.detect_green(imageXY,imageYZ)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3 / np.sqrt(dist)

    def detect_joint_pos(self, imageXZ, imageYZ):
        self.joints_pos[0] = self.detect_yellow(imageXZ, imageYZ)
        self.joints_pos[1] = self.detect_blue(imageXZ, imageYZ)
        self.joints_pos[2] = self.detect_green(imageXZ, imageYZ)
        self.joints_pos[3] = self.detect_red(imageXZ, imageYZ)
        
        return
    
    # detect robot end-effector from the image
    def detect_end_effector(self,imageXY, imageYZ):
        a = self.pixel2meter(imageXY, imageYZ)
        endPos = a * (self.detect_yellow(imageXY, imageYZ) - self.detect_red(imageXY, imageYZ))
        return endPos

    def detect_joint_angles(self):
        def x2q_joint4(z, a, b, c):
            x = z[0]
            y = z[1]
            z = z[2]

            F = np.empty((3))
            F[0] = 3*np.cos(x)*np.sin(z) + 3*np.sin(x)*np.sin(y)*np.cos(z) - a
            F[1] = 3*np.sin(x)*np.sin(z) - 3*np.cos(x)*np.sin(y)*np.cos(z) - b
            F[2] = 3*np.cos(y)*np.cos(z) + 2 - c
            return F
        
        def x2q_end(m, a, y, z):
            m = m[0]
            
            F = np.empty((1))
            F[0] = -2*np.sin(y)*np.sin(m) + 2*np.cos(y)*np.cos(z)*np.cos(m) +3*np.cos(y)*np.cos(z) + 2 - a

            return F
        
        self.detect_joints_pos()
        joint4_pos = np.array(self.joints_pos[2])
        end_pos = np.array(self.joints_pos[3])
        prev_ang = self.joints_ang
        k1 = least_squares(x2q_joint4, prev_ang[:3], args = (joint4_pos), 
                          bounds = (np.array(prev_ang[0:3]) - 0.05, np.array(prev_ang[0:3]) + 0.05))
        curr_ang = k1.x
        k2 = least_squares(x2q_end, prev_ang[3], args = ([end_pos[2], curr_ang[1], curr_ang[2]]), 
                          bounds = (prev_ang[3] - 0.05, prev_ang[3] + 0.05))
        curr_ang.append(k2.x[0])
        self.joints_ang = curr_ang
        return curr_ang
        
        
        
        pass
    def forward_kinematics(self):
        #Forward kinematics
        t1, t2, t3, t4 = sym.symbols('t1 t2 t3 t4')
        ex = 2*sym.sin(t1)*sym.cos(t2)*sym.sin(t4) - 2*sym.sin(t1)*sym.sin(t2)*sym.cos(t3)*sym.cos(t4) + 2*sym.cos(t1)*sym.sin(t3)*sym.cos(t4) + 3*sym.cos(t1)*sym.sin(t3) + 3*sym.sin(t1)*sym.sin(t2)*sym.cos(t3)
        ey = -2*sym.cos(t1)*sym.cos(t2)*sym.sin(t4) + 2*sym.sin(t1)*sym.sin(t3)*sym.cos(t4) - 2*sym.cos(t1)*sym.sin(t2)*sym.cos(t3)*sym.cos(t4) +3*sym.sin(t1)*sym.sin(t3) - 3*sym.cos(t1)*sym.sin(t2)*sym.cos(t3)
        ez = -2*sym.sin(t2)*sym.sin(t4) + 2*sym.cos(t2)*sym.cos(t3)*sym.cos(t4) + 3*sym.cos(t2)*sym.cos(t3) + 2

        return [ex, ey, ez]

    # Calculate the robot Jacobian
    def calculate_jacobian(self,imageXY, imageYZ):
        FK = self.forward_kinematics()
        joint_angles = self.detect_joint_angles()
        t1, t2, t3, t4 = sym.symbols('t1 t2 t3 t4')
        t = [t1, t2, t3, t4]
        jacobian = np.eye(3, 4)
        for i in len(FK):
            for j in len(joint_angles):
                expr = sym.diff(FK[i], t[j])
                jacobian[i][j] = expr.subs([(t1, joint_angles[0]), (t2, joint_angles[1], (t3, joint_angles[2], (t4, joint_angles[3])))])
        return jacobian


    def target_tragectory(self):
        # pulll targe move from topics
        return np.array([0,0,0])

    def control_closed(self,imageXY, imageYZ):
        # P gain
        K_p = np.array([[10,0],[0,10]])
        # D gain
        K_d = np.array([[0.1,0],[0,0.1]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        # robot end-effector position
        pos = self.detect_end_effector(imageXY, imageYZ)
        # desired trajectory
        pos_d= self.target_tragectory() 
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error)/dt
        # estimate error
        self.error = pos_d-pos
        q = self.detect_joint_angles(imageXY, imageYZ) # estimate initial value of joints'
        J_inv = np.linalg.pinv(self.calculate_jacobian(imageXY, imageYZ))  # calculating the psudeo inverse of Jacobian
        dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        return q_d

    def callback(self, cam1_data,cam2_data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(cam1_data, "bgr8")
            self.cv_image1 = self.bridge.imgmsg_to_cv2(cam2_data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # corr = self.detect_red(self.cv_image1, self.cv_image2)
        # print ("red: x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))

        # corr = self.detect_green(self.cv_image1, self.cv_image2)
        # print ("green:x {}, y {}, z1 {}".format(corr[0], corr[1], corr[2]))

        # corr = self.detect_blue(self.cv_image1, self.cv_image2)
        # print ("blue:x {}, y {}, z {}".format(corr[0], corr[1], corr[2]))

        # corr = self.detect_yellow(self.cv_image1, self.cv_image2)
        # print ("yellow:x {}, y {}, z {} ".format(corr[0], corr[1], corr[2]))

        # print(self.detect_target(self.cv_image1, self.cv_image2))

        im1=cv2.imshow('window1', self.cv_image1)
        im2=cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)
        # print(self.detect_end_effector(self.cv_image1, self.cv_image2))

    

    
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


