#ifndef VP9_STREAMERS_H_
#define VP9_STREAMERS_H_

#include <image_transport/image_transport.h>
#include "web_video_server/libav_streamer.h"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"

namespace web_video_server
{

class Vp9Streamer : public LibavStreamer
{
public:
  Vp9Streamer(const async_web_server_cpp::HttpRequest& request, async_web_server_cpp::HttpConnectionPtr connection,
              ros::NodeHandle& nh);
  ~Vp9Streamer();
protected:
  virtual void initializeEncoder();
};

class Vp9StreamerType : public LibavStreamerType
{
public:
  Vp9StreamerType();
  virtual boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest& request,
                                                           async_web_server_cpp::HttpConnectionPtr connection,
                                                           ros::NodeHandle& nh);
};

}

#endif
