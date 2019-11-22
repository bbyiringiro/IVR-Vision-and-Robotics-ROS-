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
        rospy.init_node('data_processing', anonymous=True)
        cam1_sub = message_filters.Subscriber("cam1_data", Float64MultiArray)
        cam2_sub = message_filters.Subscriber("cam2_data", Float64MultiArray)
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        sync = message_filters.TimeSynchronizer([cam1_sub, cam2_sub], 2)
        sync.registerCallback(self.callback)
        self.bridge = CvBridge()
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64') 
        self.error = np.array([0.0,0.0, 0.0], dtype='float64')  
        self.error_d = np.array([0.0,0.0,0.0], dtype='float64') 
        self.joints_ang = np.array([0.0, 0.0, 0.0, 0.0], dtype='float64') 
        self.joints_pos = np.array([[0.0, 0.0, 0.0],
                                    [0.0, 0.0, 2.0],
                                    [0.0, 0.0, 5.0],
                                    [0.0, 0.0, 7.0]], dtype='float64')
        self.prev_yellow_pos = np.array([0.0,0.0,0.0], dtype='float64')
        self.prev_blue_pos = np.array([0.0,0.0,0.0], dtype='float64')
        self.prev_green_pos = np.array([0.0,0.0,0.0], dtype='float64')
        self.prev_red_pos = np.array([0.0,0.0,0.0], dtype='float64')
        self.prev_target_pos =np.array([0.0,0.0,0.0], dtype='float64')


        # target_x = message_filters.Subscriber("/target/x_position_controller/command", Float64)
        # target_y = message_filters.Subscriber("/target/y_position_controller/command", Float64)
        # target_z = message_filters.Subscriber("/target/z_position_controller/command", Float64)
        # target_x1 = message_filters.Subscriber("/target2/x2_position_controller/command", Float64)
        # target_y2 = message_filters.Subscriber("/target2/y2_position_controller/command", Float64)
        # target_z = message_filters.Subscriber("/target2/x2_position_controller/command", Float64)
    #     sync1 = message_filters.ApproximateTimeSynchronizer([target_x, target_y, target_z],5,0.1, allow_headerless=True)
    #     sync1.registerCallback(self.callback2)
    #     self.actual_sphere_x=1.0
    #     self.actual_sphere_y=1.0
    #     self.actual_sphere_z=1.0
    
    # def callback2(self,x, y, z):
    #     self.actual_sphere_x=x.data
    #     self.actual_sphere_y=y.data
    #     self.actual_sphere_z=z.data


        

        


    def detect_blue(self,imageXZ,imageYZ):
        
        return np.array([cx, cy, cz])

    def detect_yellow(self,imageXZ,imageYZ):

        return np.array([cx, cy, cz])

    def detect_red(self,imageXZ,imageYZ):

        return np.array([cx, cy, cz])

    def detect_green(self,imageXZ,imageYZ):

        return np.array([cx, cy, cz])

    def detect_target(self, imageXZ, imageYZ):
        
        self.prev_target_pos = result
        return result
    
    def pixel2meter(self,imageXZ, imageYZ):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(imageXZ, imageYZ)
      circle2Pos = self.detect_green(imageXZ,imageYZ)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3 / np.sqrt(dist)

    def pos_normal(self, origin, joints_pos):
        for joint in joints_pos:
            joint[0] = joint[0] - origin[0]
            joint[1] = joint[1] - origin[1]
            joint[2] = origin[2] - joint[2]
        return np.array(joints_pos)

    def detect_joints_pos(self, imageXZ, imageYZ):
        a = self.pixel2meter(imageXZ, imageYZ)
        print('a: ', a)
        joint_pos0 = self.detect_yellow(imageXZ, imageYZ)
        joint_pos1 = self.detect_blue(imageXZ, imageYZ)
        joint_pos2 = self.detect_green(imageXZ, imageYZ)
        joint_pos3 = self.detect_red(imageXZ, imageYZ)
        joints_pos = self.pos_normal(joint_pos0, [joint_pos0, joint_pos1, joint_pos2, joint_pos3])
        print('joints_pos: ', joints_pos)
        return a * joints_pos
    
    # detect robot end-effector from the image
    def detect_end_effector(self,imageXZ, imageYZ):
        a = self.pixel2meter(imageXZ, imageYZ)
        origin = self.detect_yellow(imageXZ, imageYZ)
        end = self.detect_red(imageXZ, imageYZ)
        endPos = a * self.pos_normal(origin, [end])
        print('end_pos: ', endPos[0])
        return endPos[0]

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
            F[0] = -2*np.sin(y)*np.sin(m) + 2*np.cos(y)*np.cos(z)*np.cos(m) + 3*np.cos(y)*np.cos(z) + 2 - a

            return F
        
        joint4_pos = np.array(self.joints_pos[2])
        end_pos = np.array(self.joints_pos[3])
        prev_ang = self.joints_ang
        k1 = least_squares(x2q_joint4, prev_ang[:3], args = (joint4_pos),
                          bounds = (np.array(prev_ang[0:3]) - 0.05, np.array(prev_ang[0:3]) + 0.05))
        curr_ang = k1.x
        k2 = least_squares(x2q_end, prev_ang[3], args = ([end_pos[2], curr_ang[1], curr_ang[2]]),
                          bounds = (prev_ang[3] - 0.05, prev_ang[3] + 0.05))
        curr_ang = np.append(curr_ang, k2.x[0])
        self.joints_ang = curr_ang
        return curr_ang
        
        
    def forward_kinematics(self):
        #Forward kinematics
        t1, t2, t3, t4 = sym.symbols('t1 t2 t3 t4')
        ex = 2*sym.sin(t1)*sym.cos(t2)*sym.sin(t4) - 2*sym.sin(t1)*sym.sin(t2)*sym.cos(t3)*sym.cos(t4) + 2*sym.cos(t1)*sym.sin(t3)*sym.cos(t4) + 3*sym.cos(t1)*sym.sin(t3) + 3*sym.sin(t1)*sym.sin(t2)*sym.cos(t3)
        ey = -2*sym.cos(t1)*sym.cos(t2)*sym.sin(t4) + 2*sym.sin(t1)*sym.sin(t3)*sym.cos(t4) - 2*sym.cos(t1)*sym.sin(t2)*sym.cos(t3)*sym.cos(t4) +3*sym.sin(t1)*sym.sin(t3) - 3*sym.cos(t1)*sym.sin(t2)*sym.cos(t3)
        ez = -2*sym.sin(t2)*sym.sin(t4) + 2*sym.cos(t2)*sym.cos(t3)*sym.cos(t4) + 3*sym.cos(t2)*sym.cos(t3) + 2

        return [ex, ey, ez]

    # Calculate the robot Jacobian
    def calculate_jacobian(self,imageXZ, imageYZ):
        FK = self.forward_kinematics()
        joint_angles = self.detect_joint_angles()
        t1, t2, t3, t4 = sym.symbols('t1 t2 t3 t4')
        t = [t1, t2, t3, t4]
        jacobian = np.eye(3, 4)
        for i in range(len(FK)):
            for j in range(len(joint_angles)):
                expr = sym.diff(FK[i], t[j])
                jacobian[i][j] = expr.subs([(t1, joint_angles[0]), (t2, joint_angles[1]), (t3, joint_angles[2]), (t4, joint_angles[3])])
        return jacobian


    def control_closed(self,imageXZ, imageYZ):
        # P gain
        K_p = np.array([[0.3,0,0],[0,0,0.3], [0,0,0.3]])
        # D gain
        K_d = np.array([[0.1,0,0.0],[0,0.1,0.0], [0,0.0,0.1]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        # robot end-effector position
        pos = self.detect_end_effector(imageXZ, imageYZ)
        # desired trajectory
        pos_d = self.detect_target(imageXZ, imageYZ) 
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error)/dt
        # estimate error
        self.error = pos_d - pos
        q = self.joints_ang 
        J_inv = np.linalg.pinv(self.calculate_jacobian(imageXZ, imageYZ))  # calculating the psudeo inverse of Jacobian
        dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        return q_d

    def callback(self, cam1_data,cam2_data):
        try:
            self.cam1_data= self.bridge.imgmsg_to_cv2(cam1_data, "bgr8")
            self.cam2_data = self.bridge.imgmsg_to_cv2(cam2_data, "bgr8")
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

        # print(self.detect_target(self.cv_image1, self.cv_image2))
        # print(self.detect_end_effector(self.cv_image1, self.cv_image2))
        # self.joints_pos = self.detect_joints_pos(self.cv_image1, self.cv_image2)
        # q_d = self.control_closed(self.cv_image1, self.cv_image2)
        # self.joints_ang = q_d



        #q_d = self.control_open(cv_image)
        # self.joint1=Float64()
        # self.joint1.data= q_d[0]
        # self.joint2=Float64()
        # self.joint2.data= q_d[1]
        # self.joint3=Float64()
        # self.joint3.data= q_d[2]
        # self.joint4=Float64()
        # self.joint4.data= q_d[3]



        # Publish the results
        try: 
            # self.robot_joint1_pub.publish(self.joint1)
            # self.robot_joint2_pub.publish(self.joint2)
            # self.robot_joint3_pub.publish(self.joint3)
            # self.robot_joint4_pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)
    

    
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


