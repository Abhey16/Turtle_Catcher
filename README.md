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
