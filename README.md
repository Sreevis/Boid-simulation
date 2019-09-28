# Boid-simulation

Boid flocking simulation using gazebo simulator and ros

Installation Instructions
-------------------------
1. Install [ROS](https://wiki.ros.org/kinetic/Installation/Ubuntu) and gazebo
2. Setup ROS workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
```
3. Compile the workspace of boid package
```
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/Sreevis/boid-simulation.git
$ catkin_make

4. Source for `setup.bash` file for every new terminal
$ source devel/setup.bash
```

To run the simulation
-----------

Launch the gazebo simulator with boid models.

```
$ roslaunch boid_gazebo swarm_in_gazebo.launch
```

Launch boid controller node

```
$ roslaunch boid_gazebo flocking.launch
```

To set another goal

```
rostopic pub /boid/set_goal geometry_msgs/Point "x: 3.0
y: 6.0
z: 9.0" 
```

## References
[1] http://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/
