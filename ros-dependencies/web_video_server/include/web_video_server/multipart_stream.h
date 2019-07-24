#ifndef MULTIPART_STREAM_H_
#define MULTIPART_STREAM_H_

#include <ros/ros.h>
#include <async_web_server_cpp/http_connection.hpp>

namespace web_video_server
{

class MultipartStream {
public:
  MultipartStream(async_web_server_cpp::HttpConnectionPtr& connection, const std::string& boundry="boundarydonotcross");

  void sendInitialHeader();
  void sendPartHeader(const ros::Time &time, const std::string& type, size_t payload_size);
  void sendPartFooter();
  void sendPartAndClear(const ros::Time &time, const std::string& type, std::vector<unsigned char> &data);
  void sendPart(const ros::Time &time, const std::string& type, const boost::asio::const_buffer &buffer,
		async_web_server_cpp::HttpConnection::ResourcePtr resource);

private:
  async_web_server_cpp::HttpConnectionPtr connection_;
  std::string boundry_;
};

}

#endif
