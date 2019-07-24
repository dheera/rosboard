#ifndef CPP_WEB_SERVER_HTTP_CONNECTION_HPP
#define CPP_WEB_SERVER_HTTP_CONNECTION_HPP

#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include "async_web_server_cpp/http_request_handler.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_request_parser.hpp"

namespace async_web_server_cpp
{

class HttpConnection;
typedef boost::shared_ptr<HttpConnection> HttpConnectionPtr;
typedef boost::weak_ptr<HttpConnection> HttpConnectionWeakPtr;

/**
 *  Represents a connection to a client
 * The connection to the client is maintained as long as there is a shared
 * pointer to the connection object. If supplying the response is simple then
 * it can be done in the request handler callback from the server. However,
 * if the response will take time to generate or must be supplied over a long
 * period of time then a shared_ptr to the connection can be held and used
 * later. While reading and writing from multiple threads is supported, care
 * should be taken to ensure that proper external synchronization is done as
 * needed. For example, while write can be called from two threads, if two
 * calls to write need to be unseperated then calls to write should be locked
 * to prevent interleaving of different write calls.
 */
class HttpConnection : public boost::enable_shared_from_this<HttpConnection>,
  private boost::noncopyable
{
public:
  typedef boost::function<void(const char* begin, const char* end)> ReadHandler;
  typedef boost::shared_ptr<const void> ResourcePtr;

  explicit HttpConnection(boost::asio::io_service &io_service,
                          HttpServerRequestHandler request_handler);

  boost::asio::ip::tcp::socket &socket();

  /**
   * Start async operation to read request (normally called by server)
   */
  void start();

  /**
   * Perform an async read
   */
  void async_read(ReadHandler callback);

  /**
   * Write the given bytes to the socket and clear the vector
   */
  void write_and_clear(std::vector<unsigned char> &data);

  void write(const std::string &);

  void write(const boost::asio::const_buffer &buffer,
             ResourcePtr resource);

  void write(const std::vector<boost::asio::const_buffer> &buffer,
             ResourcePtr resource);

private:
  void handle_read(const char* begin, const char* end);
  void handle_read_raw(ReadHandler callback,
                       const boost::system::error_code &e,
                       std::size_t bytes_transferred);

  // Must be called while holding write lock
  void write_pending();

  void handle_write(const boost::system::error_code &e,
                    std::vector<ResourcePtr> resources);

  boost::asio::io_service::strand strand_;
  boost::asio::ip::tcp::socket socket_;
  HttpServerRequestHandler request_handler_;
  boost::array<char, 8192> buffer_;
  HttpRequest request_;
  HttpRequestParser request_parser_;

  boost::mutex write_mutex_;
  bool write_in_progress_;
  std::vector<boost::asio::const_buffer> pending_write_buffers_;
  std::vector<ResourcePtr> pending_write_resources_;
  boost::system::error_code last_error_;
  ReadHandler read_handler_;
};

}

#endif
