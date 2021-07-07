# ROSboard 

View ROS topic output from a web interface. Mobile friendly. Works on ROS1 and ROS2.

## ROS1 setup

1. Prerequisites:

```
sudo apt install python3-pip
sudo pip3 install rospkg
sudo pip3 install tornado
```

2. Run `./configure-ros1` after cloning this repo. It should now be a valid ROS1 package. Throw it inside your catkin workspace, then run `catkin_make`, `source devel/setup.bash`, the usual stuff.

3. `rosrun rosboard rosboard_node` or put it in your launch file

4. Point your web browser at http://localhost:8888

## ROS2 setup

1. Prerequisites:

```
sudo pip3 install tornado
```

2. Run `./configure-ros2` after cloning this repo. It should now be a valid ROS2 package. Throw it inside your colcon workspace, then run `colcon build`, `source install/setup.bash`, the usual stuff.

3. `ros2 run rosboard rosboard_node` or put it in your launch file

4. Point your web browser at http://localhost:8888
