# rvo_ros

## Dependencies

* Tested on Ubuntu 18.04 and ROS Melodic
* Currently depends on [freight robot gazebo](https://github.com/kenembanisi/Socially-Aware-Cooperative-Robot-Navigation) for URDF, Gazebo-dependent files, config files, etc.
* `pygame_viz.py` depends on Python 3 and `pygame`


## How to Run

1. Run the following in a sourced environment:
```
roslaunch rvo_ros example.launch
```

2. To visualize the recorded data, run the following
```
cd ~/catkin_ws/src/<navigate to package>/rvo_ros/scripts
python pygame_viz.py --data <filename>
```


## TODOs

1. ~~Be able to pass data filename via commandline argument to `pygame_viz.py`~~
2. ~~Store data of obstacles in `data_logger.py` and enable way to easy visualize~~ 
3. Explore another way to compare velocity vectors (using L2 norm is probably not the best)
4. Reread paper on RVO to consider how time horizon affects performance
5. How can we incorporate social constraints to RVO? 
6. How to deal with arbitrarily shaped static features? Using a preprocess to filter velocities using static map.