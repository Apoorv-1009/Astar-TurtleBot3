#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry

import numpy as np
import cv2
import heapq
import time
from math import dist

class AStarController(Node):
    def __init__(self):
        super().__init__('astar_controller')

        # Define robot parameters
        self.robot_params()
        # Define map parameters
        self.map_params()

        # Start the A* path planning
        self.astar()

        # Counter to publish path inputs
        self.i = 0
        # Counter to publish the same path inputs for a few iterations
        self.counter = 0
        self.reached = False
        # self.x_start, self.y_start, self.theta_start = 500, int(2000/2), 0

        # Create Subscribers
        # Subscribe to /odom topic
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)   

        # Create Publishers
        # Publish to /cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create Timers
        # Timer for publishing to /cmd_vel
        self.controller = self.create_timer(0.1, self.controller)

    def robot_params(self):
        """Define robot parameters"""
        # Define the robot parameters
        self.WHEEL_RADIUS = 33 #mm
        self.ROBOT_RADIUS = 220 #mm
        self.WHEEL_DISTANCE = 287 #mm
        
        self.clearance = 10 + self.ROBOT_RADIUS #mm   
        
    def map_params(self):
        """Define map parameters"""

        print("Generating map...")

        # Define map parameters
        self.width = 6000 #mm
        self.height = 2000 #mm
        self.scale = 5

        clearance_color = (0, 255, 255)
        obstacle_color = (0, 0, 0)
        white = (255, 255, 255)

        # Create a black canvas
        self.canvas = np.zeros((self.height, self.width, 3), dtype="uint8")
        # Create a white rectangle
        self.canvas = cv2.rectangle(self.canvas, (self.clearance, self.clearance),
                                   (self.width-self.clearance, self.height-self.clearance), white, -1)

        # OBSTACLE 1
        x1, x2 = 1500, 1750
        y1, y2 = 0, 1000
        # Draw the clearance
        for i in range(x1-self.clearance, x2+self.clearance):
            for j in range(y1+self.clearance, y2+self.clearance):
                self.canvas[j, i] = clearance_color
        # Draw the obstacle
        for i in range(x1, x2):
            for j in range(y1, y2):
                self.canvas[j, i] = obstacle_color
        
        # OBSTACLE 2
        x1, x2 = 2500, 2750
        y1, y2 = self.height-1000, self.height
        # Draw the clearance
        for i in range(x1-self.clearance, x2+self.clearance):
            for j in range(y1-self.clearance, y2-self.clearance+1):
                self.canvas[j, i] = clearance_color
        # Draw the obstacle
        for i in range(x1, x2):
            for j in range(y1, y2):
                self.canvas[j, i] = obstacle_color

        x_center, y_center = 4200, 800
        radius = 600
        # Draw the clearance
        for i in range(x_center-radius-self.clearance, x_center+radius+self.clearance):
            for j in range(y_center-radius-self.clearance, y_center+radius+self.clearance):
                if (i-x_center)**2 + (j-y_center)**2 <= (radius+self.clearance)**2 and self.canvas[j, i, 0] != 0:
                    self.canvas[j, i] = clearance_color
        # Draw the obstacle
        for i in range(x_center-radius, x_center+radius):
            for j in range(y_center-radius, y_center+radius):
                if (i-x_center)**2 + (j-y_center)**2 <= radius**2:
                    self.canvas[j, i] = obstacle_color

    def astar(self):

        # Get the class parameters
        WHEEL_RADIUS = self.WHEEL_RADIUS
        WHEEL_DISTANCE = self.WHEEL_DISTANCE
        clearance = self.clearance
        canvas = self.canvas
        width, height = self.width, self.height

        # Define the start and goal positions
        x_start, y_start, theta_start = 500, int(height/2), 0

        print("Enter goal positions with respect to bottom left corner of provided map.")

        # Get the goal positions from the user
        while True:
            x_goal = int(input('Enter goal x position (mm)' + f'({width-250}-{width-clearance-1}): '))
            y_goal = int(input('Enter goal y position (mm)' + f'({clearance}-{height-clearance-1}): '))

            y_goal = height-y_goal-1
            try:
                if canvas[y_goal, x_goal, 0] == 255 and (width-250 <= x_goal and x_goal <= width-clearance-1):
                    break
            except:
                print('Invalid input, re-enter the goal node position')
            else:
                print('Invalid input, re-enter the goal node position')

        print("Positions accepted! Calculating path...")

        self.x_start, self.y_start, self.theta_start = x_start, y_start, theta_start
        self.x_goal, self.y_goal = x_goal, y_goal

        distance_threshold = 20 #mm
        angular_threshold = 30  #deg  
        rpm1, rpm2 = 50, 100 #rpm

        # Define action set
        action_set = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), 
                      (rpm2, 0), (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1)]

        # Make a lambda function to adjust the value of x to the visited space
        adjust = lambda x, threshold: int(int(round(x*2)/2)/threshold)

        ########## IMPLEMENT A* SEARCH ALGORITHM ##########

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
        
        # Dictionary to store the inputs applied to that node
        inputs = {(x_start, y_start, theta_start): (0, 0)}

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
                self.T = 0.3
                x_new, y_new, theta_new = x, y, theta
                while t < self.T:
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
                        # Store the inputs applied to that node
                        inputs[(x_new, y_new, theta_new)] = (rpm_l, rpm_r)

                    # If the node is visited, check if the new cost is less than the previous cost
                    elif cost_to_come[(x_vis, y_vis, theta_vis)] > c2c + action_cost: 
                        parent[(x_new, y_new, theta_new)] = (x, y, theta)
                        cost_to_come[(x_vis, y_vis, theta_vis)] = c2c + action_cost 
                        cost[x_vis, y_vis, theta_vis] = cost_to_come[(x_vis, y_vis, theta_vis)] + dist((x_new, y_new), (x_goal, y_goal))
                        # Store the inputs applied to that node
                        inputs[(x_new, y_new, theta_new)] = (rpm_l, rpm_r)

        if not reached:
            print('Goal could not be reached')
            print("Exiting...")
            exit()

        ########## OPTIMAL PATH ##########
        # Get the path from the parent dictionary
        self.path = []
        # x, y = x_goal, y_goal   
        x, y, theta = x_achieved, y_achieved, theta_achieved
        while (x, y, theta) != (x_start, y_start, theta_start):
            # print(x, y)
            rpm_l, rpm_r = inputs[(x, y, theta)]
            self.path.append((x, y, theta, rpm_l, rpm_r))
            x, y, theta = parent[(x, y, theta)]
        rpm_l, rpm_r = inputs[(x, y, theta)]
        self.path.append((x, y, theta, rpm_l, rpm_r))
        self.path.reverse()

        self.path_length = len(self.path)
        # print(self.path)

        # Visualize the path
        self.visualize_path(parent, action_set)

    def visualize_path(self, parent, action_set):

        # Get the class parameters
        canvas = self.canvas
        x_start, y_start = self.x_start, self.y_start
        x_goal, y_goal = self.x_goal, self.y_goal
        WHEEL_RADIUS = self.WHEEL_RADIUS
        WHEEL_DISTANCE = self.WHEEL_DISTANCE
        path = self.path
        T = self.T
        width_resized, height_resized = int(self.width/self.scale), int(self.height/self.scale)

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
            # cv2.circle(canvas, (int(x), int(y)), 1, (255, 0, 0), 5)
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
                    x_cvs = int(x_new)
                    y_cvs = int(y_new)
                    # if clearance <= x_new < width-clearance-1 and clearance <= y_new < height-clearance-1 and canvas[y_cvs, x_cvs, 0] == 255:
                    if canvas[y_cvs, x_cvs, 0] == 255:
                        cv2.line(canvas, (int(x_parent), int(y_parent)), (x_cvs, y_cvs), (254, 0, 0), 5)
                        # cv2.circle(canvas, (int(x_cvs), int(y_cvs)), 1, (255, 0, 0), 10)
                        x_parent, y_parent = x_new, y_new
                        t += dt 
                    else:
                        break

            if(counter == threshold):
                # Resize the canvas by a factor of scale
                canvas_resized = cv2.resize(canvas, (width_resized, height_resized))
                cv2.imshow('Canvas', canvas_resized)
                # cv2.imshow('Canvas', canvas)
                cv2.waitKey(1)  
                counter = 0
            
        # Draw the start and goal nodes on the canvas
        cv2.circle(canvas, (x_start, y_start), 10, (0, 255, 0), 20)
        cv2.circle(canvas, (x_goal, y_goal), 10, (0, 165, 255), 20)

        # Draw the path on the canvas
        for i in range(len(path)-1):
            # Draw a line connecting the path points
            cv2.line(canvas, (int(path[i][0]), int(path[i][1])), (int(path[i+1][0]), int(path[i+1][1])), (0, 0, 255), 10)
            # Resize the canvas by a factor of scale
            canvas_resized = cv2.resize(canvas, (width_resized, height_resized))
            cv2.imshow('Canvas', canvas_resized)
            # cv2.imshow('Canvas', canvas)

        # Release VideoWriter
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def quaternion_to_euler(self, x, y, z, w):

        # Convert the quaternion to euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def odom_callback(self, msg):
        
        # Get the current position and orientation of the robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convert the quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

        # Store the current position and orientation
        self.x = x
        self.y = y
        self.yaw = np.degrees(yaw)

    def controller(self):

        if self.i < self.path_length:

            x_, y_, theta_, rpm_l, rpm_r = self.path[self.i]

            # Transform the coordinates to the robot frame and convert to meters
            x_ = (x_ - self.x_start) / 1000
            y_ = (self.y_start - y_) / 1000

            # Calculate the angle between the robot and the goal
            theta_ = np.rad2deg(np.arctan2(y_-self.y, x_-self.x))

            # Calculate the angular error
            yaw_error = theta_ - self.yaw

            # Calculate the position error
            distance_error = np.sqrt((self.x-x_)**2 + (self.y-y_)**2)

            # Cap the distance error to 1, to avoid high linear velocities
            distance_error = min(1, distance_error)

            # Get a twist message
            velocity_message = Twist()

            # Apply the proportional controller
            velocity_message.linear.x = 0.5 * distance_error
            velocity_message.angular.z = 0.05 * yaw_error

            # Publish the velocity message
            self.cmd_vel_pub.publish(velocity_message)

            # Print the tracking point and applied velocities
            print("x_, y_, theta_: ", np.round(x_, 3), np.round(y_, 3), np.round(theta_, 3))
            print("x, y, theta: ", np.round(self.x, 3), np.round(self.y, 3), np.round(self.yaw, 3))
            print("distance_error: ", np.round(distance_error, 3))
            print("yaw_error: ", np.round(yaw_error, 3))
            print("linear_x: ", np.round(velocity_message.linear.x, 3))
            print("angular_z: ", np.round(velocity_message.angular.z, 3))
            print("\n")
            
            if dist((self.x, self.y), (x_, y_)) < 0.1:
                self.i += 1

        else:
            # Stop the robot
            velocity_message = Twist()
            velocity_message.linear.x = 0.0
            velocity_message.angular.z = 0.0
            self.cmd_vel_pub.publish(velocity_message)
                

def main(args=None):
    rclpy.init(args=args)
    node = AStarController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    