# Turtle_Catcher

----

https://github.com/user-attachments/assets/52255c7c-ae16-48ed-af2c-ab77aac7460e

## Overview

This project implements a "Turtle_Catcher" game using ROS2 and the Turtlesim package. The goal is to create a system where one turtle catches other randomly spawned turtles in the Turtlesim environment.

### Key Components
1 Nodes:

* turtlesim_node (from Turtlesim package)
* turtle_controller (custom)
* turtle_spawner (custom)


2 Custom Interfaces:

* Turtle.msg and TurtleArray.msg
* CatchTurtle.srv

### Functionality

* Spawn turtles randomly
* Control the main turtle to catch others
* Track and update the list of alive turtles
* Implement catching mechanism

### Concepts Implemented

* ROS2 node creation and communication
* Topic publishing and subscribing
* Service client and server implementation
* Custom message and service interfaces
* Parameter handling
* Launch file creation
* Basic control algorithms (P controller)

---

## Installation
Follow these instructions to get a copy of the project up and running on your local machine for development and testing purposes.
1. Create a directory with a subfolder named **src** on your local machine.
2. Navigate into the directory using Terminal and run:
```
colcon build
```
This will convert your directory into a ROS2 Workspace

3. Clone the repository to your local machine:
```
git clone https://github.com/Abhey16/Design-and-Simulation-of-a-Mobile-Robot-with-ROS2.git
```
This will add ROS package in the **src** folder.

4. Navigate to the root of ROS2 directory and build the project using colcon:
```
colcon build
```
5. Source the environment to set up the necessary ROS2 variables:
```
source install/setup.bash
```
6. Now launch the Gazebo environment
```
ros2 launch my_robot_bringup turtle_catcher.launch.py
```
