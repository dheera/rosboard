#ifndef ROS_COMPRESSED_STREAMERS_H_
#define ROS_COMPRESSED_STREAMERS_H_

#include <sensor_msgs/CompressedImage.h>
#include "web_video_server/image_streamer.h"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "web_video_server/multipart_stream.h"

namespace web_video_server
{

class RosCompressedStreamer : public ImageStreamer
{
public:
  RosCompressedStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
			ros::NodeHandle& nh);
  virtual void start();

private:
  void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg);

  MultipartStream stream_;
  ros::Subscriber image_sub_;
};

class RosCompressedStreamerType : public ImageStreamerType
{
public:
  boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                   async_web_server_cpp::HttpConnectionPtr connection,
                                                   ros::NodeHandle& nh);
  std::string create_viewer(const async_web_server_cpp::HttpRequest &request);
};

}

#endif
