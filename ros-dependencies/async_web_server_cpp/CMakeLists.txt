cmake_minimum_required(VERSION 2.8.3)
project(async_web_server_cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system regex filesystem)
find_package(OpenSSL)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 DEPENDS Boost OpenSSL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/http_server.cpp
  src/http_connection.cpp src/http_request_parser.cpp
  src/http_reply.cpp src/http_request_handler.cpp
  src/websocket_connection.cpp src/websocket_request_handler.cpp
  src/websocket_message.cpp src/http_request.cpp)

target_link_libraries(${PROJECT_NAME}
  ${OPENSSL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)


if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED rospy roslib)
  find_package(rostest)
  add_subdirectory(test)
endif()
