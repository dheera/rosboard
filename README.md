# ROSboard 

ROS node that runs a web server on your robot.
Run the node, point your web browser at http://your-robot-ip:8888/ and you get nice visualizations.

**ROS1/ROS2 compatible.** This package will work in either ROS version.

**Mobile friendly.** Designed so you can walk around next to your robot with a phone while viewing ROS topics.

**Light weight.** Doesn't depending on much. Consumes extremely little resources when it's not actually being used.

You can run it on your desktop too and play a ROS bag.

Also be sure to check out my terminal visualization tool, [ROSshow](https://github.com/dheera/rosshow/).

![screenshot](/screenshots/screenshot1.jpg?raw=true "screenshot")

## ROS1 setup

1. Prerequisites:

```
sudo apt install python3-pip
sudo pip3 install rospkg
sudo pip3 install tornado
```

2. Run `./configure-ros1` after cloning this repo. It should now be a valid ROS1 package. Throw it inside your catkin workspace, then run `catkin_make`, `source devel/setup.bash`, the usual stuff.

3. `rosrun rosboard rosboard_node` or put it in your launch file

4. Point your web browser at http://localhost:8888 (or replace localhost with your robot's IP)

## ROS2 setup

1. Prerequisites:

```
sudo pip3 install tornado
```

2. Run `./configure-ros2` after cloning this repo. It should now be a valid ROS2 package. Throw it inside your colcon workspace, then run `colcon build`, `source install/setup.bash`, the usual stuff.

3. `ros2 run rosboard rosboard_node` or put it in your launch file

4. Point your web browser at http://localhost:8888 (or replace localhost with your robot's IP)
