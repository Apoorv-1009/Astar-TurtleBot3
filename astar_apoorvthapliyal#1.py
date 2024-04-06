# Link to github: https://github.com/Apoorv-1009/Astar-TurtleBot3/tree/main

import numpy as np
import cv2
import heapq
import time
from math import dist

########## STEP 0: TAKE INPUT FROM THE USER ##########

clearance = int(input('Enter the clearance: '))
rpm1 = int(input('Enter the rpm1: '))
rpm2 = int(input('Enter the rpm2: '))

########## STEP 1: DEFINE THE ACTION SET ##########

WHEEL_RADIUS = 33 #/1000 # 33mm
ROBOT_RADIUS = 220 #/1000 # 220mm
WHEEL_DISTANCE = 287 #/1000 # 287mm
# clearance = 10
clearance += ROBOT_RADIUS
# rpm1, rpm2 = 25, 50
distance_threshold = min(rpm1, rpm2) / 2
angular_threshold = min(rpm1, rpm2) / 2
# angular_threshold = 30

action_set = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), 
              (rpm2, 0), (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1)]

########## STEP 2: MATHEMATICAL REPRESENTATION OF FREE SPACE ##########
width = 6000
height = 2000
scale = 5

clearance_color = (0, 255, 255)
obstacle_color = (0, 0, 0)

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

# Resize the canvas by a factor of scale
width_resized = int(width/scale)
height_resized = int(height/scale)
canvas_resized = cv2.resize(canvas, (width_resized, height_resized))

########## STEP 3: IMPLEMENT STAR TO SEARCH THE TREE AND FIND THE OPTIMAL PATH ##########

# Enter the start and goal nodes with bottom left as origin
# Take input from the user, till its not in the obstacle space
while True:
    x_start = int(input('Enter start x position' + f'({clearance}-{width-clearance-1}): '))
    y_start = int(input('Enter start y position' + f'({clearance}-{height-clearance-1}): '))
    theta_start = int(input('Enter start theta position (+180 to -180): '))

    y_start = height-y_start-1
    try:
        if canvas[y_start, x_start, 0] == 255 and 180 >= theta_start >= -180:
            break
    except:
        print('Invalid input, re-enter the start node position')
    else:
        print('The start node is in the obstacle space, re-enter the goal node position')

while True:
    x_goal = int(input('Enter goal x position' + f'({clearance}-{width-clearance-1}): '))
    y_goal = int(input('Enter goal y position' + f'({clearance}-{height-clearance-1}): '))

    y_goal = height-y_goal-1
    try:
        if canvas[y_goal, x_goal, 0] == 255:
            break
    except:
        print('Invalid input, re-enter the goal node position')
    else:
        print('The goal node is in the obstacle space, re-enter the goal node position')

print("Positions accepted! Calculating path...")

# x_start, y_start, theta_start = clearance+1, clearance+1, 0
# x_goal, y_goal = width-clearance-1, clearance+1
# x_goal, y_goal = width-clearance-1, height-clearance+1

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
        T = 0.4
        x_new, y_new, theta_new = x, y, theta
        while t < T:
            dx_dt = WHEEL_RADIUS/2 * (ul + ur) * np.cos(np.radians(theta_new))
            dy_dt = WHEEL_RADIUS/2 * (ul + ur) * np.sin(np.radians(theta_new))
            dtheta_dt = np.rad2deg(WHEEL_RADIUS/WHEEL_DISTANCE * (ur - ul))

            # Save the current state
            x_prev, y_prev, theta_prev = x_new, y_new, theta_new

            # Get the new state
            x_new += dx_dt * dt
            y_new += dy_dt * dt
            theta_new += dtheta_dt * dt 

            # Check if the new state is in the obstacle space
            if canvas[int(round(y_new*2)/2), int(round(x_new*2)/2), 0] == 255:
                # Calculate the total distance travelled
                d += np.sqrt( (dx_dt*dt)**2 + (dy_dt*dt)**2)
                t += dt 
            # If the new state is in the obstacle space, revert to the previous state
            else:
                x_new, y_new, theta_new = x_prev, y_prev, theta_prev
                break

        # Let the action cost be a function of distance travelled
        action_cost = int(d)

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

########## STEP 4: OPTIMAL PATH ##########
    
# Get the path from the parent dictionary
path = []
# x, y = x_goal, y_goal   
x, y, theta = x_achieved, y_achieved, theta_achieved
while (x, y, theta) != (x_start, y_start, theta_start):
    # print(x, y)
    path.append((x, y))
    x, y, theta = parent[(x, y, theta)]
path.append((x, y))
path.reverse()

########## STEP 5: REPRESENT THE OPTIMAL PATH ##########

# Start a video writer in mp4 format
astar = cv2.VideoWriter('astar.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 50, (width_resized, height_resized))

# Draw the start and goal nodes on the canvas
cv2.circle(canvas, (x_start, y_start), 10, (0, 255, 0), 20)
cv2.circle(canvas, (x_goal, y_goal), 10, (0, 165, 255), 20)

# Draw on every threshold frame
threshold = 200
counter = 0

# Draw the visited nodes on the canvas as a curve going from the parent to the child
for x, y, theta in parent:
    counter += 1
    # Plot this point on the canvas
    cv2.circle(canvas, (int(x), int(y)), 1, (255, 0, 0), 10)
    # Plot the curve from the parent to the child
    for rpm_l, rpm_r in action_set:
        ul = 2 * np.pi * rpm_l / 60
        ur = 2 * np.pi * rpm_r / 60
        # Apply these velocities for T seconds to the model
        t = 0
        dt = 0.1
        d = 0
        x_new, y_new, theta_new = x, y, theta
        x_parent, y_parent = x, y
        while t < T:
            dx_dt = WHEEL_RADIUS/2 * (ul + ur) * np.cos(np.radians(theta_new))
            dy_dt = WHEEL_RADIUS/2 * (ul + ur) * np.sin(np.radians(theta_new))
            dtheta_dt = np.rad2deg(WHEEL_RADIUS/WHEEL_DISTANCE * (ur - ul))
            
            # Get the new state
            x_new += dx_dt * dt
            y_new += dy_dt * dt
            theta_new += dtheta_dt * dt 
            # Plot this point on the canvas
            x_cvs = int(round(x_new*2)/2)
            y_cvs = int(round(y_new*2)/2)
            # x_cvs = int(x_new)
            # y_cvs = int(y_new)
            # if clearance <= x_new < width-clearance-1 and clearance <= y_new < height-clearance-1 and canvas[y_cvs, x_cvs, 0] == 255:
            if canvas[y_cvs, x_cvs, 0] == 255:
                cv2.line(canvas, (int(x_parent), int(y_parent)), (x_cvs, y_cvs), (254, 0, 0), 5)
                x_parent, y_parent = x_new, y_new
                t += dt 
            elif canvas[y_cvs, x_cvs, 0] == 254:
                cv2.line(canvas, (int(x_parent), int(y_parent)), (x_cvs, y_cvs), (254, 0, 0), 5)
                break
            else:
                break

    if(counter == threshold):
        # Resize the canvas by a factor of scale
        canvas_resized = cv2.resize(canvas, (width_resized, height_resized))
        cv2.imshow('Canvas', canvas_resized)
        # cv2.imshow('Canvas', canvas)
        astar.write(canvas_resized)
        cv2.waitKey(1)  
        counter = 0
    
# Draw the start and goal nodes on the canvas
cv2.circle(canvas, (x_start, y_start), 10, (0, 255, 0), 20)
cv2.circle(canvas, (x_goal, y_goal), 10, (0, 165, 255), 20)

# Draw the path on the canvas
for i in range(len(path)-1):
    # Draw a red dot at path points
    # cv2.circle(canvas, (int(path[i][0]), int(path[i][1])), 1, (0, 0, 255), 15)
    # Draw a line connecting the path points
    cv2.line(canvas, (int(path[i][0]), int(path[i][1])), (int(path[i+1][0]), int(path[i+1][1])), (0, 0, 255), 10)
    # Resize the canvas by a factor of scale
    canvas_resized = cv2.resize(canvas, (width_resized, height_resized))
    cv2.imshow('Canvas', canvas_resized)
    # cv2.imshow('Canvas', canvas)
    astar.write(canvas_resized)

# Write the final frame a few times
for i in range(100):
    astar.write(canvas_resized)

# Release VideoWriter
astar.release()
cv2.waitKey(0)
cv2.destroyAllWindows()
