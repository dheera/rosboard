#ifndef CPP_WEB_SERVER_HTTP_SERVER_HPP
#define CPP_WEB_SERVER_HTTP_SERVER_HPP

#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

#include "async_web_server_cpp/http_request_handler.hpp"
#include "async_web_server_cpp/http_connection.hpp"

namespace async_web_server_cpp
{

/**
 * @class HttpServer
 * The HttpServer is an implementation of a HTTP server that serves http request from a given port
 * The server maintains a pool of threads to use to serve requests. Each request is dispatched to
 * the given request handler to be handled.
 */
class HttpServer : private boost::noncopyable
{
public:
  HttpServer(const std::string &address, const std::string &port,
             HttpServerRequestHandler request_handler, std::size_t thread_pool_size);
  ~HttpServer();

  void run();

  void stop();

private:
  void start_accept();

  void handle_accept(const boost::system::error_code &e);

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::acceptor acceptor_;
  std::size_t thread_pool_size_;
  std::vector<boost::shared_ptr<boost::thread> > threads_;
  boost::shared_ptr<HttpConnection> new_connection_;
  HttpServerRequestHandler request_handler_;
};

}

#endif
