#ifndef CPP_WEB_SERVER_HTTP_REPLY_HPP
#define CPP_WEB_SERVER_HTTP_REPLY_HPP

#include <vector>
#include <string>
#include <boost/asio.hpp>
#include "async_web_server_cpp/http_header.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request_handler.hpp"
#include <boost/filesystem.hpp>

namespace async_web_server_cpp
{

class ReplyBuilder;

/**
 *  Utility methods for constructing replys
 */
struct HttpReply
{
  enum status_type
  {
    switching_protocols = 101,
    ok = 200,
    created = 201,
    accepted = 202,
    no_content = 204,
    multiple_choices = 300,
    moved_permanently = 301,
    moved_temporarily = 302,
    not_modified = 304,
    bad_request = 400,
    unauthorized = 401,
    forbidden = 403,
    not_found = 404,
    internal_server_error = 500,
    not_implemented = 501,
    bad_gateway = 502,
    service_unavailable = 503
  } status;

  static std::vector<boost::asio::const_buffer> to_buffers(const std::vector<HttpHeader> &headers);

  /**
   * Create a request handler that sends a stock reply based on the stats code
   */
  static HttpServerRequestHandler stock_reply(status_type status);

  /**
   * Create a request handler that sends the contents of a file
   */
  static HttpServerRequestHandler from_file(HttpReply::status_type status,
      const std::string& content_type,
      const std::string& filename,
      const std::vector<HttpHeader>& additional_headers = std::vector<HttpHeader>());

  /**
   * Create a request handler that reads files from the filesystem
   * No content type is served and it is left to the browser to determine the content type
   * @param path_root the prefix in the request path that should be ignored
   * @param filesystem_root the path to search for the requested file
   */
  static HttpServerRequestHandler from_filesystem(HttpReply::status_type status,
      const std::string& path_root,
      const std::string& filesystem_root,
      bool list_directories,
      const std::vector<HttpHeader>& additional_headers = std::vector<HttpHeader>());

  /**
   * Create a request handler that sends a static response
   */
  static HttpServerRequestHandler static_reply(status_type status,
      const std::string& content_type,
      const std::string& content,
      const std::vector<HttpHeader>& additional_headers = std::vector<HttpHeader>());

  /**
   * Create a builder to create and send reply headers
   */
  static ReplyBuilder builder(status_type status);
};

/**
 * Object to build and send reply headers
 */
class ReplyBuilder
{
public:
  ReplyBuilder(HttpReply::status_type status);

  /**
   * Add a header to the reply
   */
  ReplyBuilder &header(const std::string &name, const std::string &value);

  /**
   * Add a header to the reply
   */
  ReplyBuilder &header(const HttpHeader &header);

  /**
   * Add a group of headers to the reply
   */
  ReplyBuilder &headers(const std::vector<HttpHeader> &headers);

  /**
   * Send the headers over the connection
   */
  void write(HttpConnectionPtr connection);

private:
  HttpReply::status_type status_;
  boost::shared_ptr<std::vector<HttpHeader> > headers_;
};


/**
 *  Request Handler that serves a predefined response
 */
class StaticHttpRequestHandler
{
public:
  StaticHttpRequestHandler(HttpReply::status_type status,
                           const std::vector<HttpHeader> &headers,
                           const std::string &content);

  bool operator()(const HttpRequest &, boost::shared_ptr<HttpConnection>, const char* begin, const char* end);

private:
  ReplyBuilder reply_builder_;
  const std::string content_string_;
};

/**
 *  Request Handler that serves a response from a file
 */
class FileHttpRequestHandler
{
public:
  FileHttpRequestHandler(HttpReply::status_type status,
			 const std::string& filename,
			 const std::vector<HttpHeader>& headers);

  bool operator()(const HttpRequest &, boost::shared_ptr<HttpConnection>, const char* begin, const char* end);

private:
  HttpReply::status_type status_;
  std::vector<HttpHeader> headers_;
  std::string filename_;
};

/**
 *  Request Handler that serves a responses from the filesystem from a base path
 */
class FilesystemHttpRequestHandler
{
public:
  FilesystemHttpRequestHandler(HttpReply::status_type status,
			       const std::string& path_root,
			       const std::string& filesystem_root,
			       bool list_directories,
			       const std::vector<HttpHeader>& headers);

  bool operator()(const HttpRequest &, boost::shared_ptr<HttpConnection>, const char* begin, const char* end);

private:
  HttpReply::status_type status_;
  std::vector<HttpHeader> headers_;
  std::string path_root_;
  boost::filesystem::path filesystem_root_;
  bool list_directories_;
};

}

#endif
