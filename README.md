#ENPH353 ROS & GAZEBO LAB

##Objectives
- Learn how to create simulated worlds
- Learn how to create simulated agents
- Understand how to launch basic ROS systems
- Control

##Tasks
1. Setup a ROS workspace and clone ENPH353 lab resources.
2. Create a simple simulated track for line following.
3. Create a differential drive robot.
4. Launch a ROS system to test controlling the robot.
5. Integrate computer vision based line detection in a ROS node for a PID line follower.
6. Demonstrate the robot is able to line follow and complete three laps of your track.

##Steps
###Workspace setup

Setup a ROS workspace from the command line.
```
cd ~
mkdir -p enph353_ws/src
cd enph353_ws
catkin_make
```

Whenever working with ROS, one must source the active workspace in each terminal to ensure path and environment variables are set. Source the relevant workspace when you open a new terminal window or tab.

`source ~/enph353_ws/devel/setup.bash`

This process is also documented on the ROS wiki's [create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) tutorial. For even more tutorials check the [ROS wiki](http://wiki.ros.org/ROS/Tutorials).

Clone the ENPH353 ROS & Gazebo lab repository.
```
cd ~/enph353_ws/src
git clone https://github.com/ENPH353/enph353_ROS_lab.git
```