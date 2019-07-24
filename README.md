# ROS Turtle Controller
Small and simple Turtle controller in C++ for turtlesim ROS package.

## Building 
For building, use your terminal and type following commands:

* Go to your catkin repository:
```
cd ~/PATH_TO_YOUR_CATKIN_FOLDER/
```
* Build the package:
```
catkin build ROS_Turtle_Controller --cmake-args -GEclipse CDT4 - Unix Makefiles -D__GXX_EXPERIMENTAL_CXX0X__=1 -D__cplusplus=201103L
```

## Running the Package
Implemented are two slightly different controller. 

You can start either one in your terminal with the launch file and arguments:
```
roslaunch ROS_Turtle_Controller turtle.launch controller:="1"
```
or for the PID controller:
```
roslaunch ROS_Turtle_Controller turtle.launch controller:="2"
```
By default, Controller one is selected.

For giving a Goal, you can use the Service Server, e.g. for traveling to (x,y) = (1,1):
```
rosservice call /interface/set_goal "x: 1.0 y: 1.0" 
```
#### Have fun :)
