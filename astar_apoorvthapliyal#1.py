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

########## STEP 3: IMPLEMENT STAR TO SEARCH THE TREE AND FIND THE OPTIMAL PATH ##########

x_start, y_start, theta_start = clearance+1, clearance+1, 0
x_goal, y_goal = width-clearance-1, clearance+1
# x_goal, y_goal = width-clearance-1, height-clearance+1

distance_threshold = 20
angular_threshold = 30
dt = 0.1

# Make a lambda function to adjust the value of x to the visited space
adjust = lambda x, threshold: int(int(round(x*2)/2)/threshold)

q = []
heapq.heappush(q, (0, x_start, y_start, theta_start))

# Dictionary to store visited nodes
visited = {(adjust(x_start, distance_threshold),
            adjust(y_start, distance_threshold),
            adjust(theta_start, angular_threshold)): 1}

# Dictionary to store the parent of each node
parent = {(x_start, y_start, theta_start): (x_start, y_start, theta_start)}

# Dictionary to store the cost to come of each node
cost_to_come = {(adjust(x_start, distance_threshold),
                 adjust(y_start, distance_threshold),
                 adjust(theta_start, angular_threshold)): 0}

# Dictionary to store the cost of each node
cost = {(adjust(x_start, distance_threshold),
         adjust(y_start, distance_threshold),
         adjust(theta_start, angular_threshold)): 0}

reached = False
start = time.time()

while q:
    
    _, x, y, theta = heapq.heappop(q)
    x_achieved, y_achieved, theta_achieved = x, y, theta

    # Get the cost to come of the current node
    c2c = cost_to_come[(adjust(x, distance_threshold),
                        adjust(y, distance_threshold),
                        adjust(theta, angular_threshold))]

    if dist((x, y), (x_goal, y_goal)) < 10:
        end = time.time()
        print("Goal reached")
        # Print time in minutes and seconds
        print("Time taken: ", int((end-start)/60), "minutes", int((end-start)%60), "seconds")
        # print("Goal reached: ", end-start, "seconds")
        reached = True
        x_achieved, y_achieved, theta_achieved = x, y, theta
        break

    for rpm_l, rpm_r in action_set:

        # Convert the rpm values to angular velocity
        ul = 2 * np.pi * rpm_l / 60
        ur = 2 * np.pi * rpm_r / 60

        # Apply these velocities for t seconds to the model
        t = 0
        dt = 0.1
        d = 0
        x_new, y_new, theta_new = x, y, theta
        while t < 0.4:
            dx_dt = WHEEL_RADIUS/2 * (ul + ur) * np.cos(np.radians(theta_new))
            dy_dt = WHEEL_RADIUS/2 * (ul + ur) * np.sin(np.radians(theta_new))
            dtheta_dt = np.rad2deg(WHEEL_RADIUS/WHEEL_DISTANCE * (ur - ul))

            # Get the new state
            x_new += dx_dt * dt
            y_new += dy_dt * dt
            theta_new += dtheta_dt * dt 

            # Calculate the total distance travelled
            d += np.sqrt( (dx_dt*dt)**2 + (dy_dt*dt)**2)
            t += dt 

        # d = dist((x_new, y_new), (x, y))
        # Let the action cost be a function of distance travelled
        action_cost = int(d)

        # print("x_new: ", x_new, "y_new: ", y_new, "theta_new: ", theta_new, "ul: ", ul, "ur: ", ur)

        # Keep the theta_newing angle within 180 and -180
        if theta_new > 180:
            theta_new -= 360
        elif theta_new < -180:
            theta_new += 360

        # Cap the new node values within the boundaries of the canvas
        x_new = max(clearance, min(width-clearance, x_new))
        y_new = max(clearance, min(height-clearance, y_new))

        # Adjust the values for the canvas
        x_cvs = int(round(x_new*2)/2)
        y_cvs = int(round(y_new*2)/2)
        theta_cvs = int(round(theta_new*2)/2)

        # Adjust the values for the visited dictionary
        x_vis = adjust(x_new, distance_threshold)
        y_vis = adjust(y_new, distance_threshold)
        theta_vis = adjust(theta_cvs, angular_threshold)

        # Check if the new node is within the boundaries of the canvas
        if 0 <= x_new < width and 0 <= y_new < height and canvas[y_cvs, x_cvs, 0] == 255:

            # Check if the new node is not visited
            if (x_vis, y_vis, theta_vis) not in visited:
                # Store the parent of the node
                parent[(x_new, y_new, theta_new)] = (x, y, theta)
                # Store the cost to come of the node
                cost_to_come[(x_vis, y_vis, theta_vis)] = c2c + action_cost
                # Store the cost of the node
                cost[(x_vis, y_vis, theta_vis)] = cost_to_come[(x_vis, y_vis, theta_vis)] + dist((x_new, y_new), (x_goal, y_goal))
                # Push the node into the priority queue
                heapq.heappush(q, (cost[(x_vis, y_vis, theta_vis)], x_new, y_new, theta_new))
                # Mark the node as visited
                visited[(x_vis, y_vis, theta_vis)] = 1

            # If the node is visited, check if the new cost is less than the previous cost
            elif cost_to_come[(x_vis, y_vis, theta_vis)] > c2c + action_cost: 
                parent[(x_new, y_new, theta_new)] = (x, y, theta)
                cost_to_come[(x_vis, y_vis, theta_vis)] = c2c + action_cost 
                cost[x_vis, y_vis, theta_vis] = cost_to_come[(x_vis, y_vis, theta_vis)] + dist((x_new, y_new), (x_goal, y_goal))

if not reached:
    print('Goal could not be reached')
    print("Exiting...")
    exit()

