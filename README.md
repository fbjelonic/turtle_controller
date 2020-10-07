# ROS Turtle Controller

Small Turtle controller in C++ for turtlesim ROS package. This package currently works for ROS Kinetic only.
If you are interested in learning more about this and many more projects, **check out my [website](https://www.fbjelonic.com/)!** You can also find an **example video of this project and some deeper explanations** there.
You can also find the **link to this project** directly **[here](https://www.fbjelonic.com/projects/crazy_turtle/)**

## Building 
For building, use your terminal and type following commands:

* Go to your catkin repository:
```
cd ~/PATH_TO_YOUR_CATKIN_FOLDER/
```
* Build the package:
```
catkin build turtle_controller --cmake-args -GEclipse CDT4 - Unix Makefiles -D__GXX_EXPERIMENTAL_CXX0X__=1 -D__cplusplus=201103L
```

## Running the Package
Implemented are two slightly different controller. 

* You can start either one in your terminal with the launch file and arguments:
```
roslaunch turtle_controller turtle.launch controller:="1"
```
* For the PID controller use:
```
roslaunch turtle_controller turtle.launch controller:="2"
```
By default, Controller one is selected.

* For giving a Goal, open a new Terminal. You can use the Service Server from the interface node, e.g. for traveling to (x,y) = (1,1):
```
rosservice call /interface/set_goal "x: 1.0 y: 1.0" 
```
#### Have fun :)
