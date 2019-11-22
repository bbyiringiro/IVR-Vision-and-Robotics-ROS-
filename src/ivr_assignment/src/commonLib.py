import numpy as np
import cv2
import os
import glob


def detect_red(image):
    mask = cv2.inRange(image, (0, 0, 100), (100, 100, 255))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M1 = cv2.moments(mask)
    cx = int(M1['m10'] / M1['m00'])
    cy = int(M1['m01'] / M1['m00'])

    return [cx, cy]

def detect_blue(image):
    
    mask= cv2.inRange(image, (100, 0, 0), (255, 0, 0))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M1 = cv2.moments(mask)
    cx = int(M1['m10'] / M1['m00'])
    cy = int(M1['m01'] / M1['m00'])

    return [cx, cy]

def detect_yellow(image):
    mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M1 = cv2.moments(mask)
    cx = int(M1['m10'] / M1['m00'])
    cy = int(M1['m01'] / M1['m00'])

    return [cx, cy]

def detect_green(image):
    mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M1 = cv2.moments(mask)
    cx = int(M1['m10'] / M1['m00'])
    cy = int(M1['m01'] / M1['m00'])

    return [cx, cy]

def detect_target(img):
    lower_orange = np.array([10, 20, 0],np.uint8)
    upper_orange = np.array([27, 255, 255],np.uint8)

    src = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(src, lower_orange, upper_orange)

    template_list= []
    templates = glob.glob(os.path.join('templates','template*.png'))
    

    for file_name in templates:
        tm = cv2.imread(file_name,0)
        template_list.append(tm)

    if(len(template_list)==0):
         print("No templates to match")
         quit(1)

    largestVal = 0
    largestLeft = 0
    largestShape = template_list[0].shape
    for tmp in template_list:
        result = cv2.matchTemplate(mask, tmp, cv2.TM_CCOEFF)
        _, max_val, _, max_loc = cv2.minMaxLoc(result)
        top_left = max_loc
        if(max_val>largestVal):
            largestLeft = top_left
            largestVal = max_val
            largestShape = tmp.shape
            

    cx,cy = largestLeft
    
    corridinate = [cx+int(largestShape[0]/2), cy+int(largestShape[1]/2)]

    return corridinate



