// Copyright (c) 2019 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <psen_scan/ros_parameter_handler.h>
#include "psen_scan/get_ros_parameter_exception.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include "psen_scan/decrypt_password_exception.h"
#include <arpa/inet.h>
#include <algorithm>

namespace psen_scan
{

/**
 * @brief Construct a new Ros Parameter Handler:: Ros Parameter Handler object
 *
 * Gets all Parameters from Parameter Server.
 *
 * @param nh Nodehandle from which parameters will be fetched
 *
 * @throws PSENScanFatalException
 */
RosParameterHandler::RosParameterHandler(const ros::NodeHandle& nh)
:nh_(nh),
frame_id_("scanner"),
skip_(0),
angle_start_(0),
angle_end_(2750),
publish_topic_("scan")
{
  try
  {
    // required parameters first
    int host_udp_port   = getParamFromNh<int>(nh_, "host_udp_port");

    if ( host_udp_port < 0 )
    {
      throw PSENScanFatalException("Parameter host_udp_port may not be negative!");
    }

    password_           = getParamFromNh<std::string>(nh_, "password");

    try
    {
      password_ = decryptPassword(password_);
    }
    catch(const DecryptPasswordException& e)
    {
      throw PSENScanFatalException("Invalid Password: " + std::string(e.what()));
    }

    host_udp_port_      = htole32(static_cast<uint32_t>(host_udp_port));
    std::string host_ip = getParamFromNh<std::string>(nh_, "host_ip");
    host_ip_            = htobe32(inet_network(host_ip.c_str()));
    sensor_ip_          = getParamFromNh<std::string>(nh_, "sensor_ip");

    // non-required parameters last
    float angle_start = getParamFromNh<float>(nh_, "angle_start") * 10; // Convert to tenths of degree
    if ( angle_start < 0 )
    {
      throw PSENScanFatalException("Parameter angle_start may not be negative!");
    }
    angle_start_ = static_cast<uint16_t>(angle_start);

    float angle_end = getParamFromNh<float>(nh_, "angle_end") * 10; // Convert to tenths of degree
    if ( angle_end < 0 )
    {
      throw PSENScanFatalException("Parameter angle_end may not be negative!");
    }
    angle_end_ = static_cast<uint16_t>(angle_end);

    int skip = getParamFromNh<int>(nh_, "skip");
    if ( skip < 0 )
    {
      throw PSENScanFatalException("Parameter skip may not be negative!");
    }
    skip_ = static_cast<uint16_t>(skip);

    frame_id_           = getParamFromNh<std::string>(nh_, "frame_id");
    publish_topic_      = getParamFromNh<std::string>(nh_, "publish_topic");
  }
  catch(const GetROSParameterException& e)
  {
    ROS_WARN_STREAM(e.what());
    bool required = ROS_PARAMETER.find(e.getKey())->second;

    if( required )
    {
      ROS_FATAL("Cannot proceed with default configuration.");
      throw PSENScanFatalException("Reading of required parameter failed!");
    }
    else
    {
      ROS_WARN_STREAM("Proceeding with default configuration.");
    }
  }

}

/**
 * @brief Getter method for password_
 *
 * @return std::string
 */
std::string RosParameterHandler::getPassword() const
{
  return password_;
}

/**
 * @brief Getter method for host_ip_
 *
 * @return uint32_t
 */
uint32_t RosParameterHandler::getHostIP() const
{
  return host_ip_;
}

/**
 * @brief Getter method for host_udp_port_
 *
 * @return uint32_t
 */
uint32_t RosParameterHandler::getHostUDPPort() const
{
  return host_udp_port_;
}

/**
 * @brief Getter Method for sensor_ip_
 *
 * @return std::string
 */
std::string RosParameterHandler::getSensorIP() const
{
    return sensor_ip_;
}

/**
 * @brief Getter method for frame_id_
 *
 * @return std::string
 */
std::string RosParameterHandler::getFrameID() const
{
  return frame_id_;
}

/**
 * @brief Getter method for skip_
 *
 * @return uint16_t
 */
uint16_t RosParameterHandler::getSkip() const
{
  return skip_;
}

/**
 * @brief Getter Method for angle_start_
 *
 * @return uint16_t
 */
uint16_t RosParameterHandler::getAngleStart() const
{
    return angle_start_;
}

/**
 * @brief Getter Method for angle_end_
 *
 * @return uint16_t
 */
uint16_t RosParameterHandler::getAngleEnd() const
{
  return angle_end_;
}

/**
 * @brief Getter method for publish_topic_
 *
 * @return std::string
 */
std::string RosParameterHandler::getPublishTopic() const
{
  return publish_topic_;
}




/**
 * @brief Gets one ROS-parameter from parameter-server
 *
 * @tparam T Type of parameter to fetch
 * @param nh Nodehandle through which parameter should be fetched
 * @param key Key for the parameter on parameter-server
 * @return T Parameter from parameter-server
 *
 * @throws GetROSParameterException
 */
template<class T>
T RosParameterHandler::getParamFromNh(const ros::NodeHandle& nh, const std::string& key) const
{
  T ret;
  if(!nh.getParam(key, ret))
  {
    throw GetROSParameterException("Parameter " + key + " doesn't exist on Parameter Server.", key);
  }
  return ret;
}

/**
 * @brief Decrypt password
 *
 * @param encrypted_password Encrypted Password
 * @return Decrypted Password
 *
 * @throws DecryptPasswordException
 */
std::string RosParameterHandler::decryptPassword(const std::string& encrypted_password)
{
  const int ENCRYPTED_CHAR_LEN = 2;
  const int ENCRYPTED_CHAR_BASE = 16;
  const int ADDITION_COEFF = 100; // arbitrary
  const int NUMBER_OF_ASCII_CHARS = 256;
  const int ENCRYPTION_XOR_KEY = 0xCD; // arbitrary

  std::string decrypted_password = "";

  std::string encrypted_pw_temp = encrypted_password;

  encrypted_pw_temp.erase
  (
    std::remove
    (
      encrypted_pw_temp.begin(), encrypted_pw_temp.end(), ' '
    ), encrypted_pw_temp.end()
  );

  if(encrypted_pw_temp.length() % 2 != 0)
  {
    throw DecryptPasswordException("Password length must be even!");
  }

  for(unsigned int i = 0; i < encrypted_pw_temp.length(); i += ENCRYPTED_CHAR_LEN)
  {
    char c;
    try
    {
      c =  (
                  std::stoi
                  (
                    encrypted_pw_temp.substr(i, ENCRYPTED_CHAR_LEN), nullptr, ENCRYPTED_CHAR_BASE
                  ) - i * ADDITION_COEFF / ENCRYPTED_CHAR_LEN
                ) % NUMBER_OF_ASCII_CHARS ^ ENCRYPTION_XOR_KEY;
    }
    catch(const std::invalid_argument& e)
    {
      throw DecryptPasswordException(e.what());
    }
    catch(const std::out_of_range& e)
    {
      throw DecryptPasswordException(e.what());
    }

    if(c < 32)
    {
      throw DecryptPasswordException("Control characters not allowed!");
    }

    decrypted_password += c;
  }

  return decrypted_password;
}

}