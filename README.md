# Path Planning Project 3: A-Star Algorithm in Gazebo using ROS

## Authors

- [Suriya Suresh](https://www.github.com/theunknowninfinite)
- [Aashrita Chemakura](https://github.com/aashrita-chemakura)

## Project Goals

1. Implementation of the A-star Algorithm
for a Mobile Robot.
2. Running the simulation of the A-star Algorithm in Gazebo using ROS 

## Python Packages Used 
The project utilizes the following libraries:
- numpy
- math
- matplotlib.pyplot as plt
- time
- heapq
- cv2
- rospy

## Setting Up the Project

Clone the GitHub repository to get started:

```bash
$ git clone https://github.com/aashrita-chemakura/PathPlanning_Project3_Phase2.git
```

## Running the Project
### Part One: A-star Pathfinding

To run part one, navigate to part one and run 
```bash
$ python3 pro3p2_a_star_jayasuriya_Aashrita.py
```
After execution, an OpenCV window will display the map, explored nodes, and the final backtracked path.

### Part Two: ROS and Gazebo Simulation
1. Ensure a ROS Catkin workspace is set up and the necessary turtlebot packages are installed.
2. Copy the ROS package into the src folder of your Catkin workspace.
3. Compile the workspace:
```bash 
$ cd ..
$ catkin_make
```
4. Launch the project in Gazebo:
```bash
$ roslaunch a_star_3d project.launch
```
5. Execute the pathfinding script:
```` 
$ python3 prog.py
````
Enter the requested inputs when prompted. After the path is calculated, it is displayed on a 2D map and simulated in Gazebo.

## Test Cases 
### Test Case 1:
````
start point: 10 20  
start theta: 30  
end point:150 0  
rpm1,rpm2: 1,2  
clearance: 5
````
### Test Case 2:
````
start point: 20 70  
start theta: 0  
end point:450 30  
rpm1,rpm2: 1,2  
clearance: 5  
````
## Video 
Videos are in the respective folders for part one and part two 

## Notes:
* Smaller RPM values for wheels are recommended 
* The map has to be closed for the explored nodes to be animated for Part 1.

## Support
For support, open a issue on Github.
