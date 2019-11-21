import cv2
import numpy as np
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
            # print(len(contours_xy))
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
            # print(len(contours_yz))
            print(err)
            # print (M["m10"])
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



img1 = cv2.imread('1.png', 1)
img2 = cv2.imread('s2.png', 1)
detect_target(img1, img2)