cmake_minimum_required(VERSION 2.8.3)
project(rosauth)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation rostest)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Authentication.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS roscpp message_runtime)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(ros_mac_authentication src/ros_mac_authentication.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ros_mac_authentication
  ${catkin_LIBRARIES} crypto
)

add_dependencies(ros_mac_authentication ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ros_mac_authentication
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  ## Add gtest based cpp test target and link libraries
  add_executable(ros_mac_authentication_test test/ros_mac_authentication_test.cpp)
  add_dependencies(ros_mac_authentication_test ${PROJECT_NAME}_gencpp)
  target_link_libraries(ros_mac_authentication_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} crypto dl pthread)
  add_rostest(test/ros_mac_authentication.test)
endif()
