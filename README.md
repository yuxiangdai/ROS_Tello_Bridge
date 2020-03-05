# ROS_Tello
ROS Tello driver 
Goal: for use with keyboard teleop and ROStopic command

Fork of https://github.com/yuokamoto/ros_tello

To Run: 
```
./setup.sh
catkin build
roslaunch ros_tello tello.launch
```

# Ros Topics
## tello/keys
Keyboard operations (WASD, Up/Down, Left/Right)

## tello/command
A command from the list below

```python
takeoff
land
forward
backward
left
right
ccw
cw
```