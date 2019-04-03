cmake_minimum_required(VERSION 2.8.3)
project(rosshow)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_python_setup()
catkin_package()

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_install_python(PROGRAMS
  nodes/rosshow
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

