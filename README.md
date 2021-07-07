# ROSboard 

View ROS topic output from a web interface. Mobile friendly. Works on ROS1 and ROS2.

## ROS1 setup

1. Prerequisites:

```
sudo apt install python3-pip
sudo pip3 install rospkg
sudo pip3 install tornado
```

2. Clone this repo inside `catkin_ws/src/` (or whatever you call your workspace)

3. Run `./configure-ros1`. It should then be a valid ROS1 package.

4. `catkin_make; source devel/setup.bash` (usual stuff)

5. `rosrun rosboard rosboard_node` (or add it to your launch file)

## ROS2 setup

1. Prerequisites:

```
sudo pip3 install tornado
```

2. Clone this repo inside `colcon_ws/src/` (or whatever you call your workspace).

3. Run `./configure-ros2`. It should then be a valid ROS2 package.

4. `colcon build; source install/setup.bash` (usual stuff)

5. `ros2 run rosboard rosboard_node` (or add it to your launch file)
