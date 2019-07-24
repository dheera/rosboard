#ifndef CPP_WEB_SERVER_WEBSOCKET_MESSAGE_HPP
#define CPP_WEB_SERVER_WEBSOCKET_MESSAGE_HPP

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/logic/tribool.hpp>
#include <boost/tuple/tuple.hpp>

namespace async_web_server_cpp
{

class WebsocketMessage;

/**
 * An single websocket frame and associated header
 */
class WebsocketFrame
{
public:
  struct Header
  {
    enum opcode
    {
      opcode_continuation = 0,
      opcode_text = 1,
      opcode_binary = 2,
      opcode_close = 8,
      opcode_ping = 9,
      opcode_pong = 10,
    } opcode : 4;
    bool rsv3 : 1;
    bool rsv2 : 1;
    bool rsv1 : 1;
    bool fin : 1;

    unsigned int len : 7;
    bool mask : 1;
  } __attribute__((__packed__));
  union
  {
    Header header;
    char header_bytes[2];
  };
  uint64_t length;
  unsigned char mask[4];
  std::string content;

  bool fromMessage(const WebsocketMessage& message);
  bool serialize(std::vector<unsigned char>& buffer);
};

class WebsocketFrameParser
{
public:
  WebsocketFrameParser();
  void reset();
  boost::tribool consume(WebsocketFrame& frame, char input);
  template<typename InputIterator>
  boost::tuple<boost::tribool, InputIterator> parse(WebsocketFrame& frame,
      InputIterator begin, InputIterator end)
  {
    while (begin != end)
    {
      boost::tribool result = consume(frame, *begin++);
      if (result || !result)
        return boost::make_tuple(result, begin);
    }
    boost::tribool result = boost::indeterminate;
    return boost::make_tuple(result, begin);
  }

private:
  enum state
  {
    header_byte1,
    header_byte2,
    length_8bytes_left,
    length_7bytes_left,
    length_6bytes_left,
    length_5bytes_left,
    length_4bytes_left,
    length_3bytes_left,
    length_2bytes_left,
    length_1bytes_left,
    mask_byte1,
    mask_byte2,
    mask_byte3,
    mask_byte4,
    body
  } state_;

};

/**
 * A websocket message that in potentially constructed from/destructed to
 * a WebsocketFrame.
 */
class WebsocketMessage
{
public:
  WebsocketMessage();
  enum type
  {
    type_unknown,
    type_text,
    type_binary,
    type_close,
    type_ping,
    type_pong,
  } type;
  std::string content;
};

class WebsocketFrameBuffer
{
public:
  boost::tribool consume(WebsocketMessage& message, WebsocketFrame& frame);
};


}

#endif
