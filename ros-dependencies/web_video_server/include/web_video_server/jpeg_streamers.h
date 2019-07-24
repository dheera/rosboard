#ifndef JPEG_STREAMERS_H_
#define JPEG_STREAMERS_H_

#include <image_transport/image_transport.h>
#include "web_video_server/image_streamer.h"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "web_video_server/multipart_stream.h"

namespace web_video_server
{

class MjpegStreamer : public ImageTransportImageStreamer
{
public:
  MjpegStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
                ros::NodeHandle& nh);

protected:
  virtual void sendImage(const cv::Mat &, const ros::Time &time);

private:
  MultipartStream stream_;
  int quality_;
};

class MjpegStreamerType : public ImageStreamerType
{
public:
  boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                   async_web_server_cpp::HttpConnectionPtr connection,
                                                   ros::NodeHandle& nh);
  std::string create_viewer(const async_web_server_cpp::HttpRequest &request);
};

class JpegSnapshotStreamer : public ImageTransportImageStreamer
{
public:
  JpegSnapshotStreamer(const async_web_server_cpp::HttpRequest &request,
                       async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh);

protected:
  virtual void sendImage(const cv::Mat &, const ros::Time &time);

private:
  int quality_;
};

}

#endif
