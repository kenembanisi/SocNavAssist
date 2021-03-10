# rvo_ros


Last updated: March 10th 2021
## Dependencies

* Tested on Ubuntu 18.04 and ROS Melodic
* Currently depends on [freight robot gazebo](https://github.com/kenembanisi/Socially-Aware-Cooperative-Robot-Navigation) for URDF, Gazebo-dependent files, config files, etc.
* `pygame_viz.py` depends on Python 3 and `pygame`


## How to Run

At the moment, two scenarios have been implemented: (1) Approach and, (2) Crossing

1. Approach Scenario: Run the following in a sourced environment:
```
roslaunch rvo_ros approach.launch
```

![approach](/media/approach.gif)

***
<br>

2. Crossing Scenario: Run the following in a sourced environment:
```
roslaunch rvo_ros crossing.launch
```

![approach](/media/crossing.gif)

***
<br>

> **_NOTE:_** 
    By default, the `control_mode` is set to `auto` in the scenario launch files. 
    If you wish to use `manual` mode, include `control_mode:='manual` when running the launch file
    
    E.g.  roslaunch rvo_ros approach.launch control_mode:='manual' 

***
<br>

## Evaluation

1. To visualize the recorded data in pygame, run the following
```
cd ~/catkin_ws/src/<navigate to package>/rvo_ros/scripts
python pygame_viz.py --data <filename>
```

2. To plot the velocities from a recorded data, run the following
```
cd ~/catkin_ws/src/<navigate to package>/rvo_ros/scripts
python velocity_plotter.py --data <filename>
```


## TODOs

1. How to tune `tau` (planning horizon) to achieve optimal behavior?
2. Explore another way to compare velocity vectors (using L2 norm is probably not the best).
    - It seems that choosing the right `tau` resolves the main issues here
3. How can we incorporate social constraints to RVO? 
4. How to deal with arbitrarily shaped static features? Using a preprocess to filter velocities using static map.
5. Solve angular oscillation issue! So far, it seems the angular acceleration is a constraint.