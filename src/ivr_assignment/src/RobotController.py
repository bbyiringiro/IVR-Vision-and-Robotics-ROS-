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
        # initialize the node named data
        
        rospy.init_node('cam_data_processing', anonymous=True)
        cam1_sub = message_filters.Subscriber("/cam1_data", Float64MultiArray)
        cam2_sub = message_filters.Subscriber("/cam2_data", Float64MultiArray)
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.target_x_pub = rospy.Publisher("/target_pred_x", Float64, queue_size=10)
        self.target_y_pub = rospy.Publisher("/target_pred_y", Float64, queue_size=10)
        self.target_z_pub = rospy.Publisher("/target_pred_z", Float64, queue_size=10)
        self.end_x_pub = rospy.Publisher("/end_effector_x", Float64, queue_size=10)
        self.end_y_pub = rospy.Publisher("/end_effector_y", Float64, queue_size=10)
        self.end_z_pub = rospy.Publisher("/end_effector_z", Float64, queue_size=10)
        self.sync = message_filters.ApproximateTimeSynchronizer([cam1_sub, cam2_sub],5,0.1, allow_headerless=True)
        self.sync.registerCallback(self.callback)
        self.bridge = CvBridge()
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64') 
        self.error = np.array([0.0,0.0, 0.0], dtype='float64')  
        self.error_d = np.array([0.0,0.0,0.0], dtype='float64') 
        self.joints_ang = np.array([0.0, 0.0, 0.0, 0.0], dtype='float64') 
        self.joints_pos = np.array([[0.0, 0.0, 0.0],
                                    [0.0, 0.0, 2.0],
                                    [0.0, 0.0, 5.0],
                                    [0.0, 0.0, 7.0]], dtype='float64')
        self.pixel_to_meter = 0.037488286740304605
        self.YZ =np.array([
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0]
        ])

        self.XZ =np.array([
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0]
        ])


    def detect_blue(self,imageXZ,imageYZ):
        cx = imageXZ[0]
        cy = imageYZ[0]
        cz = (imageXZ[1] + imageYZ[1])/2
        return np.array([cx, cy, cz])

    def detect_yellow(self,imageXZ,imageYZ):
        cx = imageXZ[0]
        cy = imageYZ[0]
        cz = (imageXZ[1] + imageYZ[1])/2
        return np.array([cx, cy, cz])

    def detect_red(self,imageXZ,imageYZ):
        cx = imageXZ[0]
        cy = imageYZ[0]
        cz = (imageXZ[1] + imageYZ[1])/2
        return np.array([cx, cy, cz])

    def detect_green(self,imageXZ,imageYZ):
        cx = imageXZ[0]
        cy = imageYZ[0]
        cz = (imageXZ[1] + imageYZ[1])/2
        return np.array([cx, cy, cz])

    def detect_target(self, imageXZ, imageYZ):
        x = imageXZ[0]
        y = imageYZ[0]
        z = (imageXZ[1]+imageYZ[1])/2

        result = self.pixel_to_meter * self.pos_normal(self.detect_yellow(self.XZ[3,:], self.YZ[3,:]),  [np.array([x, y,z])])
        result[0][2] =result[0][2]+1

        return result[0]
    
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

    def detect_joints_pos(self):
        a = self.pixel_to_meter
        # print('a: ', a)
        joint_pos0 = self.detect_yellow(self.XZ[3,:], self.YZ[3,:])
        joint_pos1 = self.detect_blue(self.XZ[2,:], self.YZ[2,:])
        joint_pos2 = self.detect_green(self.XZ[1,:], self.YZ[1,:])
        joint_pos3 = self.detect_red(self.XZ[0,:], self.YZ[0,:])
        joints_pos = self.pos_normal(joint_pos0, [joint_pos0, joint_pos1, joint_pos2, joint_pos3])
        # print('joints_pos: ', joints_pos)
        return a * joints_pos
    
    # detect robot end-effector from the image
    def detect_end_effector(self):
        a = self.pixel_to_meter
        origin = self.detect_yellow(self.XZ[3,:], self.YZ[3,:])
        end = self.detect_red(self.XZ[0,:], self.YZ[0,:])
        endPos = a * self.pos_normal(origin, [end])
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
    def calculate_jacobian(self):
        FK = self.forward_kinematics()
        joint_angles = self.joints_ang
        t1, t2, t3, t4 = sym.symbols('t1 t2 t3 t4')
        t = [t1, t2, t3, t4]
        jacobian = np.eye(3, 4)
        for i in range(len(FK)):
            for j in range(len(joint_angles)):
                expr = sym.diff(FK[i], t[j])
                jacobian[i][j] = expr.subs([(t1, joint_angles[0]), (t2, joint_angles[1]), (t3, joint_angles[2]), (t4, joint_angles[3])])
        return jacobian


    def control_closed(self):
        # P gain
        K_p = np.array([[0.3, 0.0, 0.0], [0.0, 0.4, 0.0], [0.0, 0.0, 0.3]])
        # D gain
        K_d = np.array([[0.002, 0.0, 0.0],[0.0, 0.002, 0.0], [0.0, 0.0, 0.001]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        # robot end-effector position
        pos = self.detect_end_effector()
        # desired trajectory
        pos_d = self.detect_target(self.XZ[4,:], self.YZ[4,:]) 
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error)/dt
        # estimate error
        self.error = pos_d - pos
        q = self.joints_ang 
        J_inv = np.linalg.pinv(self.calculate_jacobian())  # calculating the psudeo inverse of Jacobian
        dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        return q_d

    def callback(self, cam1_data,cam2_data):
        # recieves data corrected processed by each node handling cameras
        # The data is 10 size array, each of the two represent both axes info
        # reshapes in in the order of joints row index 0 being red joing and 3th index, yellow and last target
        # [[red], [green], [blue], [yellow], [target_corridinates]]
        self.YZ = np.array(cam1_data.data).reshape(5,2)
        self.XZ = np.array(cam2_data.data).reshape(5,2)

        end_effector =self.detect_end_effector()
        target_pred = self.detect_target(self.XZ[4,:], self.YZ[4,:])
        #get and update joint position
        self.joints_pos = self.detect_joints_pos()
        # takes integration each step and update joints_angles
        q_d= self.control_closed()
        self.joints_ang = q_d

        
        self.joint1=Float64()
        self.joint1.data= q_d[0]
        self.joint2=Float64()
        self.joint2.data= q_d[1]
        self.joint3=Float64()
        self.joint3.data= q_d[2]
        self.joint4=Float64()
        self.joint4.data= q_d[3]



        # Publish the results
        try: 
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
            self.target_x_pub.publish(target_pred[0])
            self.target_y_pub.publish(target_pred[1])
            self.target_z_pub.publish(target_pred[2])
            self.end_x_pub.publish(end_effector[0])
            self.end_y_pub.publish(end_effector[1])
            self.end_z_pub.publish(end_effector[2])
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


