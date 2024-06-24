import os

if os.environ.get("ROS_VERSION") == "1":
    import rospy  # ROS1
elif os.environ.get("ROS_VERSION") == "2":
    import rosboard.rospy2 as rospy  # ROS2
else:
    print("ROS not detected. Please source your ROS environment\n(e.g. 'source /opt/ros/$ROS_DISTRO/setup.bash')")
    exit(1)