#ifndef CPP_WEB_SERVER_HTTP_REQUEST_HANDLER_HPP
#define CPP_WEB_SERVER_HTTP_REQUEST_HANDLER_HPP

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include "async_web_server_cpp/http_request.hpp"

namespace async_web_server_cpp
{

class HttpConnection;

/**
 * A handler for requests
 * Should return true if the request was successfuly handled
 * Returning false will cause the next matching handler to be triggered
 * If false is returned then nothing should be written to the connection
 */
typedef boost::function<bool(const HttpRequest &, boost::shared_ptr<HttpConnection>, const char* begin, const char* end)> HttpServerRequestHandler;

/**
 * A hander that can dispatch to a request to different handlers depending on a
 * predicate. If none of registered handlers satisfy the request then the
 * default request handler is used.
 */
class HttpRequestHandlerGroup
{
public:
  typedef boost::function<bool(const HttpRequest &)> HandlerPredicate;

  HttpRequestHandlerGroup(HttpServerRequestHandler default_handler);

  void addHandlerForPath(const std::string &path_regex, HttpServerRequestHandler handler);

  void addHandler(HandlerPredicate predicate, HttpServerRequestHandler handler);

  bool operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection, const char* begin, const char* end);

private:
  HttpServerRequestHandler default_handler_;
  std::vector<std::pair<HandlerPredicate, HttpServerRequestHandler> > handlers_;
};

class HttpRequestBodyCollector
{
public:
  typedef boost::function<void(const HttpRequest &, boost::shared_ptr<HttpConnection>, const std::string& body)> Handler;

  HttpRequestBodyCollector(Handler handler);

  bool operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection, const char* begin, const char* end);

private:
  Handler handler_;
};

}

#endif
