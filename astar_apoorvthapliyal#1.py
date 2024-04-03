import numpy as np
import cv2
import heapq
import time
from math import dist

##### STEP 1: DEFINE THE ACTION SET #####

WHEEL_RADIUS = 33 #/1000 # 33mm
ROBOT_RADIUS = 220 #/1000 # 220mm
WHEEL_DISTANCE = 287 #/1000 # 287mm
clearance = 20
clearance += ROBOT_RADIUS

rpm1, rpm2 = 50, 100

action_set = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), 
              (rpm2, 0), (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1)]

