#include "async_web_server_cpp/websocket_message.hpp"

namespace async_web_server_cpp
{

bool WebsocketFrame::fromMessage(const WebsocketMessage& message)
{
  switch (message.type)
  {
  case WebsocketMessage::type_text:
    header.opcode = WebsocketFrame::Header::opcode_text;
    break;
  case WebsocketMessage::type_binary:
    header.opcode = WebsocketFrame::Header::opcode_binary;
    break;
  case WebsocketMessage::type_close:
    header.opcode = WebsocketFrame::Header::opcode_close;
    break;
  case WebsocketMessage::type_ping:
    header.opcode = WebsocketFrame::Header::opcode_ping;
    break;
  case WebsocketMessage::type_pong:
    header.opcode = WebsocketFrame::Header::opcode_pong;
    break;
  default:
    return false;
  }
  header.fin = true;
  header.rsv1 = false;
  header.rsv2 = false;
  header.rsv3 = false;
  content = message.content;
  length = message.content.size();
  return true;
}

bool WebsocketFrame::serialize(std::vector<unsigned char>& buffer)
{
  int header_size = 2; // first two bytes of header
  if (length < 126)
  {
    header.len = length;
  }
  else if (length <= std::numeric_limits<uint16_t>::max())
  {
    header.len = 126;
    header_size += 2;
  }
  else if (length <= std::numeric_limits<uint64_t>::max())
  {
    header.len = 127;
    header_size += 8;
  }
  else
    return false; // Message too big
  header.mask = false; // sending from server

  buffer.resize(header_size + content.size());
  buffer[0] = header_bytes[0];
  buffer[1] = header_bytes[1];

  if (length < 126)
  {
    // already copied in header bytes
  }
  else if (length <= std::numeric_limits<uint16_t>::max())
  {
    buffer[2] = (length >> 8) & 0xFF;
    buffer[3] = (length >> 0) & 0xFF;
  }
  else if (length <= std::numeric_limits<uint64_t>::max())
  {
    buffer[2] = (length >> 56) & 0xFF;
    buffer[3] = (length >> 48) & 0xFF;
    buffer[4] = (length >> 40) & 0xFF;
    buffer[5] = (length >> 32) & 0xFF;
    buffer[6] = (length >> 24) & 0xFF;
    buffer[7] = (length >> 16) & 0xFF;
    buffer[8] = (length >> 8) & 0xFF;
    buffer[9] = (length >> 0) & 0xFF;
  }
  content.copy((char*)&buffer[header_size], content.size());
  return true;
}


WebsocketFrameParser::WebsocketFrameParser()
{
  reset();
}

void WebsocketFrameParser::reset()
{
  state_ = header_byte1;
}

boost::tribool WebsocketFrameParser::consume(WebsocketFrame& frame, char input)
{
  switch (state_)
  {
  case header_byte1:
    frame.header_bytes[0] = input;
    state_ = header_byte2;
    return boost::indeterminate;
  case header_byte2:
    frame.header_bytes[1] = input;
    if (frame.header.len < 126)
    {
      frame.length = frame.header.len;
      frame.content.reserve(frame.length);
      frame.content.resize(0);
      if (frame.header.mask)
        state_ = mask_byte1;
      else if (frame.length > 0)
        state_ = body;
      else
        return true;
    }
    else if (frame.header.len == 126)
    {
      frame.length = 0;
      state_ = length_2bytes_left;
    }
    else
    {
      frame.length = 0;
      state_ = length_8bytes_left;
    }
    return boost::indeterminate;

  case length_8bytes_left:
    frame.length |= ((uint64_t)(input & 0xFF) << 56);
    state_ = length_7bytes_left;
    return boost::indeterminate;
  case length_7bytes_left:
    frame.length |= ((uint64_t)(input & 0xFF) << 48);
    state_ = length_6bytes_left;
    return boost::indeterminate;
  case length_6bytes_left:
    frame.length |= ((uint64_t)(input & 0xFF) << 40);
    state_ = length_5bytes_left;
    return boost::indeterminate;
  case length_5bytes_left:
    frame.length |= ((uint64_t)(input & 0xFF) << 32);
    state_ = length_4bytes_left;
    return boost::indeterminate;
  case length_4bytes_left:
    frame.length |= ((uint64_t)(input & 0xFF) << 24);
    state_ = length_3bytes_left;
    return boost::indeterminate;
  case length_3bytes_left:
    frame.length |= ((uint64_t)(input & 0xFF) << 16);
    state_ = length_2bytes_left;
    return boost::indeterminate;
  case length_2bytes_left:
    frame.length |= ((uint64_t)(input & 0xFF) << 8);
    state_ = length_1bytes_left;
    return boost::indeterminate;
  case length_1bytes_left:
    frame.length |= ((uint64_t)(input & 0xFF) << 0);
    frame.content.reserve(frame.length);
    frame.content.resize(0);
    if (frame.header.mask)
      state_ = mask_byte1;
    else
      state_ = body;
    return boost::indeterminate;


  case mask_byte1:
    frame.mask[0] = input;
    state_ = mask_byte2;
    return boost::indeterminate;
  case mask_byte2:
    frame.mask[1] = input;
    state_ = mask_byte3;
    return boost::indeterminate;
  case mask_byte3:
    frame.mask[2] = input;
    state_ = mask_byte4;
    return boost::indeterminate;
  case mask_byte4:
    frame.mask[3] = input;
    if (frame.length > 0)
      state_ = body;
    else
      return true;
    return boost::indeterminate;

  case body:
    frame.content += input;
    if (frame.content.size() < frame.length)
      return boost::indeterminate;
    //unmask the frame
    if (frame.header.mask)
    {
      for (int i = 0; i < frame.length; ++i)
      {
        frame.content[i] = frame.content[i] ^ frame.mask[i % 4];
      }
    }
    return true;
  default:
    return false;
  }
}

WebsocketMessage::WebsocketMessage() : type(type_unknown) {}

boost::tribool WebsocketFrameBuffer::consume(WebsocketMessage& message, WebsocketFrame& frame)
{
  if (frame.header.opcode == WebsocketFrame::Header::opcode_continuation)
  {
    if (message.type == WebsocketMessage::type_unknown)
      return false;
    else
      message.content.append(frame.content);
  }
  else
  {
    switch (frame.header.opcode)
    {
    case WebsocketFrame::Header::opcode_text:
      message.type = WebsocketMessage::type_text;
      break;
    case WebsocketFrame::Header::opcode_binary:
      message.type = WebsocketMessage::type_binary;
      break;
    case WebsocketFrame::Header::opcode_close:
      message.type = WebsocketMessage::type_close;
      break;
    case WebsocketFrame::Header::opcode_ping:
      message.type = WebsocketMessage::type_ping;
      break;
    case WebsocketFrame::Header::opcode_pong:
      message.type = WebsocketMessage::type_pong;
      break;
    default:
      message.type = WebsocketMessage::type_unknown;
      return false;
    }
    message.content = frame.content;
  }
  if (frame.header.fin)
    return true;
  else
    return boost::indeterminate;
}


}
