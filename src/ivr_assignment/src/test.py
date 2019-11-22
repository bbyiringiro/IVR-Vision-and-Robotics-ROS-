import cv2
import numpy as np
import commonLib as lib


img = cv2.imread('s2.png', 1)
lib.detect_yellow(img)