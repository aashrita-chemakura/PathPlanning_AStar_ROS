# PathPlanning_Project3_Phase2



## Authors

- [Suriya Suresh](https://www.github.com/theunknowninfinite)
- [Aashrita Chemakura](https://github.com/aashrita-chemakura)

## 1. Project Goals

1. Implementation of the A-star Algorithm
for a Mobile Robot.
2. Running the simulation of the A-star Algorithm in Gazebo using ROS 


## 2. Python Packages Used 
The code uses the following libraries/modules:
* numpy 
* math
* matplotlib.pyplot as plt
* time
* heapq 
* cv2 
* rospy


## 3. Setting up the script

1. Clone the github repo.

```` 
$ git clone https://github.com/aashrita-chemakura/PathPlanning_Project3_Phase2.git
````

## 4. Running Part One 

To run part one, navigate to part one and run 
```` 
$ python3 pro3p2_a_star_jayasuriya_Aashrita.py
````
After the determined path is found, a opencv window will pop up with the map and then the explored nodes and finally the backtracked path. 
## 5. Running Part Two 
* This part utilizes ROS.
* Copy the ROS package into the src folder, with the prior assumption a Catkin Workspace has been made and the user has installed the necessary turtlebot packages.
* Run catkin_make after going to the root of the ws. 
```` 
$ cd .. 
$ catkin_make
````
* The launch file needs to be run to launch gazebo . 
```` 
$ roslaunch a_star_3d project.launch
````
* Afterwards , run the python file to get the output. 
```` 
$ python3 prog.py
````
* The code asks for inputs similar to part one. The test cases have been described in the following section.

* After the input has been entered, the path is found and displayed on a 2D Map. 
* Now the output will be run in gazebo in which the bot follows the determined path. 
## Test Cases 

### Test Case 1:
start point: 10 20  
start theta: 30  
end point:150 0  
rpm1,rpm2: 1,2  
clearance: 5
### Test Case 2:
start point: 20 70  
start theta: 0  
end point:450 30  
rpm1,rpm2: 1,2  
clearance: 5  

## Video 
Videos are in the respective folders for part one and part two 

## Notes:
* Smaller RPM values for wheels are recommended 
* The map has to be closed for the explored nodes to be animated for Part 1.

## Support
For support, open a issue on Github.





