#include "web_video_server/jpeg_streamers.h"
#include "async_web_server_cpp/http_reply.hpp"

namespace web_video_server
{

MjpegStreamer::MjpegStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageTransportImageStreamer(request, connection, nh), stream_(connection)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);
  stream_.sendInitialHeader();
}

void MjpegStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  std::vector<int> encode_params;
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  stream_.sendPartAndClear(time, "image/jpeg", encoded_buffer);
}

boost::shared_ptr<ImageStreamer> MjpegStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                                    async_web_server_cpp::HttpConnectionPtr connection,
                                                                    ros::NodeHandle& nh)
{
  return boost::shared_ptr<ImageStreamer>(new MjpegStreamer(request, connection, nh));
}

std::string MjpegStreamerType::create_viewer(const async_web_server_cpp::HttpRequest &request)
{
  std::stringstream ss;
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\"></img>";
  return ss.str();
}

JpegSnapshotStreamer::JpegSnapshotStreamer(const async_web_server_cpp::HttpRequest &request,
                                           async_web_server_cpp::HttpConnectionPtr connection,
                                           ros::NodeHandle& nh) :
    ImageTransportImageStreamer(request, connection, nh)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);
}

void JpegSnapshotStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  std::vector<int> encode_params;
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  char stamp[20];
  sprintf(stamp, "%.06lf", time.toSec());
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
      "Server", "web_video_server").header("Cache-Control",
                                           "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0").header(
      "X-Timestamp", stamp).header("Pragma", "no-cache").header("Content-type", "image/jpeg").header(
      "Access-Control-Allow-Origin", "*").header("Content-Length",
                                                 boost::lexical_cast<std::string>(encoded_buffer.size())).write(
      connection_);
  connection_->write_and_clear(encoded_buffer);
  inactive_ = true;
}

}
