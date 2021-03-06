# Astar-Turtlebot3
## Authors
```
Name: Akhilrajan Vethirajan
Uid: 117431773

Name: Vishaal Kanna Sivakumar
Uid: 117764314
```
1. Download repository and build your catkin workspace.
2. **Requirements:** 
- numpy
- matplotlib
- heapq
- rospy
- geometry_msgs (Twist())
- time
3. The launch folder contains the Gazebo-Turtlebot3 launch folder
4. The A* planner is in the nodes directory
5. All outputs are in the outputs sub directory
6. To run Part 1, go to part_1 directory and run the python script.
7. Make sure the script is executable (chmod +x <python_script_name>) 

![part1_output](https://github.com/Akhilrajan-V/Astar-Turtlebot3/blob/main/outputs/Part1%20outputs/Astar_graph.gif)

# Part 2 Turtlebot3 Execution
# To Launch from a single roslaunch file
8. open a new terminal and run,
   > roslaunch astar astar_gazebo.launch
# To Launch Gazebo environment and planner separately

9. To run the Gazebo simulation, in a new terminal run: 
   >  roslaunch astar turtlebot3.launch
10. In another terminal run the A* planner using,

   > rosrun astar path_planner.py


![til](./outputs/Part2%20outputs/astar_turtlebot_sim.gif)


11. Please keep the initial positions as follows:
```
X: 100
Y: 950
Theta: 0
```
 ***Note: Robot runs best when RPM1=30 and RPM2=40***
 # Simulation Output Videos
 ## A_star_gazebo1.mp4
 ```
 Start X = 100
 Start Y = 900
 Start Theta = 0
 
 ---------
 
 Goal X = 450
 Goal Y = 800
 
 ---------
 
 RPM1 = 30
 RPM2 = 40

 ```
  ## A_star_gazebo2.mp4
 ```
 Start X = 100
 Start Y = 950
 Start Theta = 0
 
 ---------
 
 Goal X = 800
 Goal Y = 950
 
 ---------
 
 RPM1 = 30
 RPM2 = 40
 ```
