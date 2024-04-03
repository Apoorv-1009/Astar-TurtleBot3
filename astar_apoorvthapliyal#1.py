import numpy as np
import cv2
import heapq
import time
from math import dist

########## STEP 1: DEFINE THE ACTION SET ##########

WHEEL_RADIUS = 33 #/1000 # 33mm
ROBOT_RADIUS = 220 #/1000 # 220mm
WHEEL_DISTANCE = 287 #/1000 # 287mm
clearance = 20
clearance += ROBOT_RADIUS

rpm1, rpm2 = 50, 100

action_set = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), 
              (rpm2, 0), (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1)]

########## STEP 2: MATHEMATICAL REPRESENTATION OF FREE SPACE ##########
width = 6000
height = 2000
scale = 5

clearance_color = (0, 255, 255)
obstacle_color = (0, 0, 0)
robot_radius_color = (255, 105, 180)

# Create a black canvas
canvas = np.zeros((height, width, 3), dtype="uint8")
# Create a white rectangle
canvas = cv2.rectangle(canvas, (clearance, clearance), (width-clearance, height-clearance), (255, 255, 255), -1)

# OBSTACLE 1
x1, x2 = 1500, 1750
y1, y2 = 0, 1000
# Draw the clearance
for i in range(x1-clearance, x2+clearance):
    for j in range(y1+clearance, y2+clearance):
        canvas[j, i] = clearance_color
# Draw the obstacle
for i in range(x1, x2):
    for j in range(y1, y2):
        canvas[j, i] = obstacle_color

# OBSTACLE 2
x1, x2 = 2500, 2750
y1, y2 = height-1000, height
# Draw the clearance
for i in range(x1-clearance, x2+clearance):
    for j in range(y1-clearance, y2-clearance+1):
        canvas[j, i] = clearance_color
# Draw the obstacle
for i in range(x1, x2):
    for j in range(y1, y2):
        canvas[j, i] = obstacle_color
        
x_center, y_center = 4200, 800
radius = 600
# Draw the clearance
for i in range(x_center-radius-clearance, x_center+radius+clearance):
    for j in range(y_center-radius-clearance, y_center+radius+clearance):
        if (i-x_center)**2 + (j-y_center)**2 <= (radius+clearance)**2 and canvas[j, i, 0] != 0:
            canvas[j, i] = clearance_color
# Draw the obstacle
for i in range(x_center-radius, x_center+radius):
    for j in range(y_center-radius, y_center+radius):
        if (i-x_center)**2 + (j-y_center)**2 <= radius**2:
            canvas[j, i] = obstacle_color

x_start, y_start = clearance+1, clearance+1
x_goal, y_goal = width-clearance-1, clearance+1

# Resize the canvas by a factor of scale
width_resized = int(width/scale)
height_resized = int(height/scale)
canvas_resized = cv2.resize(canvas, (width_resized, height_resized))

