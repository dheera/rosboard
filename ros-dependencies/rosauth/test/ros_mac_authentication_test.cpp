#include <fstream>
#include <gtest/gtest.h>
#include <openssl/sha.h>
#include <ros/ros.h>
#include <rosauth/Authentication.h>
#include <sstream>
#include <string>

using namespace std;
using namespace ros;

ServiceClient client;

TEST(RosHashAuthentication, validAuthentication)
{
  string secret = "abcdefghijklmnop";
  string client_ip = "192.168.1.101";
  string dest_ip = "192.168.1.111";
  string rand = "xyzabc";
  Time now = Time::now();
  string user_level = "admin";
  Time end = Time::now();
  end.sec += 120;

  // create the string to hash
  stringstream ss;
  ss << secret << client_ip << dest_ip << rand << now.sec << user_level << end.sec;
  string local_hash = ss.str();
  unsigned char sha512_hash[SHA512_DIGEST_LENGTH];
  SHA512((unsigned char *)local_hash.c_str(), local_hash.length(), sha512_hash);

  // convert to a hex string to compare
  char hex[SHA512_DIGEST_LENGTH * 2];
  for (int i = 0; i < SHA512_DIGEST_LENGTH; i++)
    sprintf(&hex[i * 2], "%02x", sha512_hash[i]);

  // make the request
  rosauth::Authentication srv;
  srv.request.mac = string(hex);
  srv.request.client = client_ip;
  srv.request.dest = dest_ip;
  srv.request.rand = rand;
  srv.request.t = now;
  srv.request.level = user_level;
  srv.request.end = end;

  EXPECT_TRUE(client.call(srv));
  EXPECT_TRUE(srv.response.authenticated);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // initialize ROS and the node
  init(argc, argv, "ros_hash_authentication_test");
  NodeHandle node;

  // setup the service client
  client = node.serviceClient<rosauth::Authentication>("authenticate");

  return RUN_ALL_TESTS();
}
