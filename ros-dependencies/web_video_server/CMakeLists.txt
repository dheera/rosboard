cmake_minimum_required(VERSION 2.8.3)
project(web_video_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roslib cv_bridge image_transport async_web_server_cpp)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(avformat libavformat REQUIRED)
pkg_check_modules(avutil libavutil REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${avformat_INCLUDE_DIRS}
  ${avutil_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/web_video_server.cpp
  src/image_streamer.cpp
  src/libav_streamer.cpp
  src/vp8_streamer.cpp
  src/h264_streamer.cpp
  src/vp9_streamer.cpp
  src/multipart_stream.cpp
  src/ros_compressed_streamer.cpp
  src/jpeg_streamers.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${avcodec_LIBRARIES}
  ${avformat_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
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

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
