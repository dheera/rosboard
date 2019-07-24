#include "web_video_server/image_streamer.h"
#include <cv_bridge/cv_bridge.h>

namespace web_video_server
{

ImageStreamer::ImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
    request_(request), connection_(connection), nh_(nh), inactive_(false)
{
  topic_ = request.get_query_param_value_or_default("topic", "");
}

ImageTransportImageStreamer::ImageTransportImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageStreamer(request, connection, nh), it_(nh), initialized_(false)
{
  output_width_ = request.get_query_param_value_or_default<int>("width", -1);
  output_height_ = request.get_query_param_value_or_default<int>("height", -1);
  invert_ = request.has_query_param("invert");
  default_transport_ = request.get_query_param_value_or_default("default_transport", "raw");
}

void ImageTransportImageStreamer::start()
{
  image_transport::TransportHints hints(default_transport_);
  ros::master::V_TopicInfo available_topics;
  ros::master::getTopics(available_topics);
  inactive_ = true;
  for (size_t it = 0; it<available_topics.size(); it++){
    if(available_topics[it].name == topic_){
      inactive_ = false;
    }
  }
  image_sub_ = it_.subscribe(topic_, 1, &ImageTransportImageStreamer::imageCallback, this, hints);
}

void ImageTransportImageStreamer::initialize(const cv::Mat &)
{
}

void ImageTransportImageStreamer::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if (inactive_)
    return;

  cv::Mat img;
  try
  {
    if (msg->encoding.find("F") != std::string::npos)
    {
      // scale floating point images
      cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
      cv::Mat_<float> float_image = float_image_bridge;
      double max_val;
      cv::minMaxIdx(float_image, 0, &max_val);

      if (max_val > 0)
      {
        float_image *= (255 / max_val);
      }
      img = float_image;
    }
    else
    {
      // Convert to OpenCV native BGR color
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    int input_width = img.cols;
    int input_height = img.rows;

    if (output_width_ == -1)
      output_width_ = input_width;
    if (output_height_ == -1)
      output_height_ = input_height;

    if (invert_)
    {
      // Rotate 180 degrees
      cv::flip(img, img, false);
      cv::flip(img, img, true);
    }

    cv::Mat output_size_image;
    if (output_width_ != input_width || output_height_ != input_height)
    {
      cv::Mat img_resized;
      cv::Size new_size(output_width_, output_height_);
      cv::resize(img, img_resized, new_size);
      output_size_image = img_resized;
    }
    else
    {
      output_size_image = img;
    }

    if (!initialized_)
    {
      initialize(output_size_image);
      initialized_ = true;
    }
    sendImage(output_size_image, msg->header.stamp);

  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR_THROTTLE(30, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (cv::Exception &e)
  {
    ROS_ERROR_THROTTLE(30, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (boost::system::system_error &e)
  {
    // happens when client disconnects
    ROS_DEBUG("system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (std::exception &e)
  {
    ROS_ERROR_THROTTLE(30, "exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (...)
  {
    ROS_ERROR_THROTTLE(30, "exception");
    inactive_ = true;
    return;
  }
}

}
