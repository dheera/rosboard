#ifndef IMAGE_STREAMER_H_
#define IMAGE_STREAMER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "async_web_server_cpp/http_server.hpp"
#include "async_web_server_cpp/http_request.hpp"

namespace web_video_server
{

class ImageStreamer
{
public:
  ImageStreamer(const async_web_server_cpp::HttpRequest &request,
		async_web_server_cpp::HttpConnectionPtr connection,
		ros::NodeHandle& nh);

  virtual void start() = 0;

  bool isInactive()
  {
    return inactive_;
  }
  ;

  std::string getTopic()
  {
    return topic_;
  }
  ;
protected:
  async_web_server_cpp::HttpConnectionPtr connection_;
  async_web_server_cpp::HttpRequest request_;
  ros::NodeHandle nh_;
  bool inactive_;
  image_transport::Subscriber image_sub_;
  std::string topic_;
};


class ImageTransportImageStreamer : public ImageStreamer
{
public:
  ImageTransportImageStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
			      ros::NodeHandle& nh);

  virtual void start();

protected:
  virtual void sendImage(const cv::Mat &, const ros::Time &time) = 0;

  virtual void initialize(const cv::Mat &);

  image_transport::Subscriber image_sub_;
  int output_width_;
  int output_height_;
  bool invert_;
  std::string default_transport_;
private:
  image_transport::ImageTransport it_;
  bool initialized_;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};

class ImageStreamerType
{
public:
  virtual boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                           async_web_server_cpp::HttpConnectionPtr connection,
                                                           ros::NodeHandle& nh) = 0;

  virtual std::string create_viewer(const async_web_server_cpp::HttpRequest &request) = 0;
};

}

#endif
