# ROSboard 

View ROS topic output from a web interface. Mobile friendly. Works on ROS1 and ROS2.

## ROS1 setup

1. Prerequisites:

```
sudo apt install python3-pip
sudo pip3 install rospkg
sudo pip3 install tornado
```

2. Clone this repo into your catkin workspace and run `./configure-ros1`. It should then be a valid ROS1 package.

3. `catkin_make; source devel/setup.bash` (usual stuff)

4. `rosrun rosboard rosboard_node` (or add it to your launch file)

## ROS2 setup

1. Prerequisites:

```
sudo pip3 install tornado
```

2. Clone this repo into your colcon workspace and run `./configure-ros2`. It should then be a valid ROS2 package.

3. `colcon build; source install/setup.bash` (usual stuff)

4. `ros2 run rosboard rosboard_node` (or add it to your launch file)
