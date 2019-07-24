#ifndef CPP_WEB_SERVER_HTTP_HEADER_HPP
#define CPP_WEB_SERVER_HTTP_HEADER_HPP

#include <string>

namespace async_web_server_cpp
{

/**
 * Represents a HTTP header in a request or response
 */
struct HttpHeader
{
  HttpHeader()
  {
  }

  HttpHeader(std::string name, std::string value) : name(name), value(value)
  {
  }

  std::string name;
  std::string value;
};

}

#endif
