#include <boost/regex.hpp>
#include <openssl/sha.h>
#include <openssl/hmac.h>
#include <openssl/evp.h>
#include <openssl/bio.h>
#include <openssl/buffer.h>
#include "async_web_server_cpp/websocket_request_handler.hpp"
#include "async_web_server_cpp/websocket_connection.hpp"
#include "async_web_server_cpp/http_reply.hpp"

namespace async_web_server_cpp
{

const std::string WebsocketHttpRequestHandler::KEY_MAGIC_STRING = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

WebsocketHttpRequestHandler::WebsocketHttpRequestHandler(WebsocketRequestHandler handler)
  : handler_(handler) {}


bool WebsocketHttpRequestHandler::operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection, const char* begin, const char* end)
{
  std::string connection_header = request.get_header_value_or_default("Connection", "");
  std::string upgrade_header = request.get_header_value_or_default("Upgrade", "");
  std::string websocket_key = request.get_header_value_or_default("Sec-WebSocket-Key", "");

  if (connection_header.find("Upgrade") != std::string::npos && upgrade_header.compare("websocket") == 0
      && websocket_key.size() > 0)
  {
    std::string concat_key = websocket_key + KEY_MAGIC_STRING;

    // compute the sha1 hash of the concatonated key
    unsigned char sha1_buf[20];
    SHA1((const unsigned char*)concat_key.data(), concat_key.size(), sha1_buf);

    // base64 encode the hash
    BIO* b64 = BIO_new(BIO_f_base64());
    BIO* bmem = BIO_new(BIO_s_mem());
    b64 = BIO_push(b64, bmem);
    BIO_write(b64, sha1_buf, 20);
    BIO_flush(b64);
    BUF_MEM *bptr;
    BIO_get_mem_ptr(b64, &bptr);
    std::string base64_key(bptr->data, bptr->length - 1);
    BIO_free_all(b64);


    async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::switching_protocols)
    .header("Upgrade", "websocket")
    .header("Connection", "Upgrade")
    .header("Sec-WebSocket-Version", "13")
    .header("Sec-WebSocket-Accept", base64_key)
    .write(connection);

    WebsocketConnectionPtr websocket_connection(new WebsocketConnection(connection));
    WebsocketConnection::MessageHandler message_handler = handler_(request, websocket_connection);
    websocket_connection->set_message_handler(message_handler);
    websocket_connection->handle_read(begin, end);
  }
  else
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::bad_request)(request, connection, begin, end);
  }
  return true;
}

}
