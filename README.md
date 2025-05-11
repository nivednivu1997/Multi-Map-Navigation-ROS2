# Multi-Map Navigation with Wormhole Transitions in ROS 2

## Overview

This project implements a **multi-map navigation system** in ROS 2 where a mobile robot can autonomously navigate between multiple separately mapped rooms using a custom "wormhole" mechanism. Each room is individually mapped, and overlapping regions between rooms (wormholes) allow the robot to transition between maps seamlessly.

### Key Features
- Navigate to a pose in the current or a different map.
- Automatically switch maps using predefined "wormhole" poses.
- Uses `nav2`'s `NavigateToPose` action client for navigation.
- Uses `nav2`'s `LoadMap` service for dynamic map loading.
- Custom action interface `MapNavigation` for goal management.
- Feedback and result handling with proper logging and error management.

## Dependencies

- ROS 2 Humble (or later)
- `nav2_msgs`
- `geometry_msgs`
- `custom_msgs` (custom message package containing `MapNavigation.action`)

## [Demonstration] (https://drive.google.com/file/d/1Emu8EpixYeHzUSP-oKnuVpvai1Fv_Ksk/view?usp=sharing): 



## Steps to run



1. Clone the repo 
```
git clone https://github.com/nivednivu1997/Multi-Map-Navigation-ROS2.git
```
2. Install Dependencies
```
rosdep install --from-path src --ignore-src -y
```
3. Building Package
```
 cd Multi-Map-Navigation-ROS2 && colon build --symlink-install
```


## Steps to run 

1. Launch the gazebo simulation
```
cd Multi-Map-Navigation-ROS2 && source install/setup.bash && ros2 launch butlerbot_rmf_gazebo simulation.launch.py
```
2. Launch the navigation2 stack 
```
cd Multi-Map-Navigation-ROS2 && source install/setup.bash && ros2 launch butlerbot_navigation navigation.launch.py
```
3. Run Multi map Action server 
```
cd cd Multi-Map-Navigation-ROS2 && source install/setup.bash && ros2 run cpp_node multi_map_node
```
4. Run the action client(format)
```
ros2 action send_goal /navigate_to_room custom_msgs/action/MapNavigation "target_pose:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
target_map: ''"



```
## Acknowledgments  

This project utilizes the Gazebo simulation setup from [ROS-Assignment](https://github.com/manojm-dev/ROS-Assignment).  




