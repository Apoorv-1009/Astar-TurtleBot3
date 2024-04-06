# Path Planning using A* Algorithm on TurtleBot3
Directory ID: apoorv10 <br>
UID: 120401565 <br>
Link to github: https://github.com/Apoorv-1009/Astar-TurtleBot3/tree/main

## Part 01: 2D Implementation
The astar_apoorvthapliyal#1.py code demonstrates path planning using the A* algorithm in a 2D grid environment and Gazebo. The algorithm finds the shortest path from a start node to a goal node while avoiding obstacles.

### How to provide inputs

1. Follow the prompts to enter the following sample inputs:
- Clearance (mm): 10
- RPM1 (Rounds-Per-Minute): 50
- RPM2 (Rounds-Per-Minute): 100
- Start x position (mm): 230
- Start y position (mm): 1769
- Start angle (theta): 0
- Goal x position (mm): 5769
- Goal y position (mm): 1769
2. If the start/goal happens to be in the obstacle space, then the user will be repromted to enter the start/goal till it is not in the obstacle space
6. The script will visualize the path planning process and display the result.

### Libraries/Dependencies Used

- NumPy: For numerical computations and array manipulation.
- OpenCV (cv2): For image processing tasks such as drawing and displaying images.
- heapq: Provides functions for heap queue operations (used for priority queue implementation in A* algorithm).
- time: For calculating time taken by code to complete execution
- math: For mathematical operations

### Description

1. **Action Set Definition:** Defines each action (movement) in the grid.

2. **Mathematical Representation of Free Space:** Defines the canvas and obstacles in the grid environment. Obstacles are represented by filled polygons.

3. **Generate the Graph and Check for Goal Node:** Implements A* algorithm to search for the optimal path from the start node to the goal node while avoiding obstacles.

4. **Optimal Path Generation:** Retrieves the optimal path from the parent dictionary generated by A* algorithm.

5. **Represent the Optimal Path:** Visualizes the start and goal nodes, visited nodes, and the optimal path on the canvas. This visualization is performed in an animated manner, showing the progression of the algorithm.

### Note
1. The script generates an output video (`astar.mp4`) showing the progression of the algorithm and the final result.
2. Video of output: https://youtu.be/XTeudTxqjBo

## Part 02: Gazebo Visualization
The turtlebot3_project3 package contains the source files for the A* algorithm in Gazebo using ROS2 on a Turtlebot3 Waffle. The algorithm finds the shortest path from the spawn position to a goal node.

### Libraries/Dependencies Used

- NumPy: For numerical computations and array manipulation.
- OpenCV (cv2): For image processing tasks such as drawing and displaying images.
- heapq: Provides functions for heap queue operations (used for priority queue implementation in A* algorithm).
- time: For calculating time taken by code to complete execution
- math: For mathematical operations

### How to Launch the nodes
After building and sourcing the workspace, run the following command:
```
ros2 launch turtlebot3_project3 competition_world.launch.py
```
In a new terminal, run the following command to start the A* node:
```
ros2 run turtlebot3_project3 astar_controller.py
```
After running the A* node, a path will be generated and visualised. After dismissing the map, the Turtlebot in the gazebo simulation will start to track the A* path using a closed loop controller.

### Note
1. The following parameters were used for the A* path generation:
- Clearance (mm): 20
- RPM1 (Rounds-Per-Minute): 50
- RPM2 (Rounds-Per-Minute): 100
- Start x position (mm): 500
- Start y position (mm): 1000
- Start angle (theta): 0
- Goal x position (mm): 5759
- Goal y position (mm): 1758 <br>
(The above parameters are defined in the map frame with bottom left as origin, and NOT in the gazebo frame) <br>
2. Video of output:
