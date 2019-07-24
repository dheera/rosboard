/*!
 * \file ros_mac_authentication.cpp
 * \brief Provides authentication via a message authentication codes (MAC).
 *
 * This node provides a service call that can be used to authenticate a user to use the ROS server.
 * The node relies on a hash string that is made up of several pieces of information and hashed
 * using the SHA-1 algorithm. This node is based on the best-practice method of a message
 * authentication codes (MAC).
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 4, 2013
 */

#include <fstream>
#include <openssl/sha.h>
#include <ros/ros.h>
#include <rosauth/Authentication.h>
#include <sstream>
#include <string>

using namespace std;
using namespace ros;

/*!
 * \def SECRET_FILE_PARAM
 * The ROS parameter name for the file that contains the secret string. We do not store the actual
 * string in the parameter server as the parameter server itself may not be secure.
 */
#define SECRET_FILE_PARAM "/ros_mac_authentication/secret_file_location"

/*!
 * \def MISSING_PARAMETER
 * Error code for a missing SECRET_FILE_PARAM ROS parameter.
 */
#define MISSING_PARAMETER -1

/*!
 * \def FILE_IO_ERROR
 * Error code for an IO error when reading the secret file.
 */
#define FILE_IO_ERROR -2

/*!
 * \def INVALID_SECRET
 * Error code for an invalid secret string.
 */
#define INVALID_SECRET -3

/*!
 * \def SECRET_LENGTH
 * Length of the secret string.
 */
#define SECRET_LENGTH 16

// the secret string used in the MAC
string secret;

bool authenticate(rosauth::Authentication::Request &req, rosauth::Authentication::Response &res)
{
  // keep track of the current time
  Time t = Time::now();
  // clocks can be out of sync, check which comes later
  Duration *diff;
  if (req.t > t)
    diff = new Duration(req.t - t);
  else
    diff = new Duration(t - req.t);
  bool time_check = diff->sec < 5 && req.end > t;
  delete diff;

  // check if we pass the time requirement
  if (time_check)
  {
    // create the string to hash
    stringstream ss;
    ss << secret << req.client << req.dest << req.rand << req.t.sec << req.level << req.end.sec;
    string local_hash = ss.str();

    // check the request
    unsigned char sha512_hash[SHA512_DIGEST_LENGTH];
    SHA512((unsigned char *)local_hash.c_str(), local_hash.length(), sha512_hash);

    // convert to a hex string to compare
    char hex[SHA512_DIGEST_LENGTH * 2];
    for (int i = 0; i < SHA512_DIGEST_LENGTH; i++)
      sprintf(&hex[i * 2], "%02x", sha512_hash[i]);

    // an authenticated user must match the MAC string
    res.authenticated = (strcmp(hex, req.mac.c_str()) == 0);
  }
  else
    res.authenticated = false;

  return true;
}

/*!
 * Creates and runs the ros_mac_authentication node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  init(argc, argv, "ros_mac_authentication");
  NodeHandle node;

  // check if we have to check the secret file
  string file;
  if (!node.getParam(SECRET_FILE_PARAM, file))
  {
    ROS_ERROR("Parameter '%s' not found.", SECRET_FILE_PARAM);
    return MISSING_PARAMETER;
  }
  else
  {
    // try and load the file
    ifstream f;
    f.open(file.c_str(), ifstream::in);
    if (f.is_open())
    {
      // should be a 1 line file with the string
      getline(f, secret);
      f.close();
      // check the length of the secret
      if (secret.length() != SECRET_LENGTH)
      {
        ROS_ERROR("Secret string not of length %d.", SECRET_LENGTH);
        return INVALID_SECRET;
      }
      else
      {
        ServiceServer service = node.advertiseService("authenticate", authenticate);
        ROS_INFO("ROS Authentication Server Started");
        spin();

        return EXIT_SUCCESS;
      }
    }
    else
    {
      ROS_ERROR("Could not read from file '%s'", file.c_str());
      return FILE_IO_ERROR;
    }
  }
}
