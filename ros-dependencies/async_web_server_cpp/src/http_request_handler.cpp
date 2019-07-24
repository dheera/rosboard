#include <boost/regex.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "async_web_server_cpp/http_request_handler.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_reply.hpp"

namespace async_web_server_cpp
{

HttpRequestHandlerGroup::HttpRequestHandlerGroup(HttpServerRequestHandler default_handler)
  : default_handler_(default_handler)
{
}

class PathMatcher
{
public:
  explicit PathMatcher(const std::string &path_regex_string)
    : path_regex_(boost::regex(path_regex_string))
  {
  }

  bool operator()(const HttpRequest &request)
  {
    return regex_match(request.path, path_regex_);
  }

private:
  const boost::regex path_regex_;
};

void HttpRequestHandlerGroup::addHandlerForPath(const std::string &path_regex, HttpServerRequestHandler handler)
{
  addHandler(PathMatcher(path_regex), handler);
}

void HttpRequestHandlerGroup::addHandler(HandlerPredicate predicate, HttpServerRequestHandler handler)
{
  handlers_.push_back(std::make_pair(predicate, handler));
}


bool HttpRequestHandlerGroup::operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection, const char* begin, const char* end)
{
  for (int i = 0; i < handlers_.size(); ++i)
  {
    std::pair<HandlerPredicate, HttpServerRequestHandler> &handler = handlers_[i];
    if (handler.first(request))
    {
      if(handler.second(request, connection, begin, end))
	return true;
    }
  }
  return default_handler_(request, connection, begin, end);
}

class BodyCollectingConnection;
typedef boost::shared_ptr<BodyCollectingConnection> BodyCollectingConnectionPtr;
typedef boost::weak_ptr<BodyCollectingConnection> BodyCollectingConnectionWeakPtr;
class BodyCollectingConnection : public boost::enable_shared_from_this<BodyCollectingConnection>,
				 private boost::noncopyable
{
public:
  BodyCollectingConnection(HttpRequestBodyCollector::Handler handler, const HttpRequest &request, boost::shared_ptr<HttpConnection> connection)
    : handler_(handler), request_(request), connection_(connection), received_length_(0) {
    std::string length_str = request_.get_header_value_or_default("Content-Length", "");
    try {
      length_ = boost::lexical_cast<ssize_t>(length_str);
    }
    catch(const boost::bad_lexical_cast &) {
      length_ = -1; //indicate error
    }
  }

  static void static_handle_read(BodyCollectingConnectionPtr _this, const char* begin, const char* end) {
    _this->handle_read(begin, end);
  }
  void handle_read(const char* begin, const char* end) {
    if(length_ < 0) {
      HttpReply::builder(HttpReply::bad_request).write(connection_);
      connection_->write("No Content-Length header");
      return;
    }
    std::string chunk(begin, end-begin);
    body_stream_ << chunk;
    received_length_ += chunk.length();
    if(received_length_ >= length_) {
      handler_(request_, connection_, body_stream_.str().substr(0, length_));
    }
    else {
      connection_->async_read(boost::bind(&BodyCollectingConnection::static_handle_read, shared_from_this(), _1, _2));
    }
  }

private:
  HttpRequestBodyCollector::Handler handler_;
  const HttpRequest request_;
  boost::shared_ptr<HttpConnection> connection_;
  std::stringstream body_stream_;
  ssize_t length_;
  size_t received_length_;
};

HttpRequestBodyCollector::HttpRequestBodyCollector(Handler handler)
  : handler_(handler) {}

bool HttpRequestBodyCollector::operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection, const char* begin, const char* end)
{
  BodyCollectingConnectionPtr collecting_connection(new BodyCollectingConnection(handler_, request, connection));
  collecting_connection->handle_read(begin, end);
  return true;
}

}
