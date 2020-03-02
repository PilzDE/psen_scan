// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
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
#include <psen_scan/default_parameters.h>
#include <psen_scan/decrypt_password_exception.h>
#include <psen_scan/psen_scan_fatal_exception.h>
#include <arpa/inet.h>
#include <gtest/gtest.h>

using namespace psen_scan;

#define DELETE_ROS_PARAM(param_name)                                                                                   \
  if (ros::param::has(param_name))                                                                                     \
  {                                                                                                                    \
    ros::param::del(param_name);                                                                                       \
  }

#define DELETE_ALL_ROS_PARAMS()                                                                                        \
  DELETE_ROS_PARAM("password");                                                                                        \
  DELETE_ROS_PARAM("sensor_ip");                                                                                       \
  DELETE_ROS_PARAM("host_ip");                                                                                         \
  DELETE_ROS_PARAM("host_udp_port");                                                                                   \
  DELETE_ROS_PARAM("angle_start");                                                                                     \
  DELETE_ROS_PARAM("angle_end");                                                                                       \
  DELETE_ROS_PARAM("frame_id");                                                                                        \
  DELETE_ROS_PARAM("skip");                                                                                            \
  DELETE_ROS_PARAM("publish_topic");                                                                                   \
  DELETE_ROS_PARAM("x_axis_rotation");

namespace psen_scan_test
{
class ROSParameterHandlerTest : public ::testing::Test
{
protected:
  ROSParameterHandlerTest()
    : password_("ac0d68d033")
    , host_ip_("1.2.3.5")
    , host_udp_port_(12345)
    , sensor_ip_("1.2.3.4")
    , expected_password_("admin")
    , expected_host_ip_(htobe32(inet_network(host_ip_.c_str())))
    , expected_host_udp_port_(htole32(host_udp_port_))
    , expected_frame_id_(DEFAULT_FRAME_ID)
    , expected_skip_(DEFAULT_SKIP)
    , expected_angle_start_(DEFAULT_ANGLE_START)
    , expected_angle_end_(DEFAULT_ANGLE_END)
    , expected_x_axis_rotation_(DEFAULT_X_AXIS_ROTATION)
    , expected_publish_topic_(DEFAULT_PUBLISH_TOPIC)
  {
  }

  // Default values to set
  std::string password_;
  std::string host_ip_;
  int host_udp_port_;
  std::string sensor_ip_;

  // Default expected values
  std::string expected_password_;
  uint32_t expected_host_ip_;
  uint32_t expected_host_udp_port_;
  std::string expected_frame_id_;
  uint16_t expected_skip_;
  PSENscanInternalAngle expected_angle_start_;
  PSENscanInternalAngle expected_angle_end_;
  Degree expected_x_axis_rotation_;
  std::string expected_publish_topic_;
};

TEST_F(ROSParameterHandlerTest, test_no_param)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  EXPECT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);
}

TEST_F(ROSParameterHandlerTest, test_required_params_only)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  ros::param::set("password", password_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);

  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle);
                  EXPECT_EQ(param_handler.getPassword(), expected_password_);
                  EXPECT_EQ(param_handler.getSensorIP(), sensor_ip_);
                  EXPECT_EQ(param_handler.getHostIP(), expected_host_ip_);
                  EXPECT_EQ(param_handler.getHostUDPPort(), expected_host_udp_port_);
                  EXPECT_EQ(param_handler.getFrameID(), expected_frame_id_);
                  EXPECT_EQ(param_handler.getSkip(), expected_skip_);
                  EXPECT_EQ(param_handler.getAngleStart(), expected_angle_start_);
                  EXPECT_EQ(param_handler.getAngleEnd(), expected_angle_end_);
                  EXPECT_EQ(param_handler.getXAxisRotation(), expected_x_axis_rotation_);
                  EXPECT_EQ(param_handler.getPublishTopic(), expected_publish_topic_););
}

TEST_F(ROSParameterHandlerTest, test_single_required_params_missing_password)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // ros::param::set("password", password_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);
}

TEST_F(ROSParameterHandlerTest, test_single_required_params_missing_sensor_ip)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  ros::param::set("password", password_);
  // ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);
}

TEST_F(ROSParameterHandlerTest, test_single_required_params_missing_host_ip)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  ros::param::set("password", password_);
  ros::param::set("sensor_ip", sensor_ip_);
  // ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);
}

TEST_F(ROSParameterHandlerTest, test_single_required_params_missing_host_udp_port)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  ros::param::set("password", password_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("host_ip", host_ip_);
  // ros::param::set("host_udp_port", host_udp_port_);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);
}

TEST_F(ROSParameterHandlerTest, test_all_params)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  int skip = 2;
  std::string frame_id = "abcdefg";
  std::string publish_topic = "zyxwvu";
  float angle_start = 10.5;
  float angle_end = 204.7;
  double x_axis_rotation = 100.3;
  ros::param::set("password", password_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("angle_start", angle_start);
  ros::param::set("angle_end", angle_end);
  ros::param::set("x_axis_rotation", x_axis_rotation);
  ros::param::set("frame_id", frame_id);
  ros::param::set("skip", skip);
  ros::param::set("publish_topic", publish_topic);

  uint16_t expected_skip = 2;
  PSENscanInternalAngle expected_angle_start(105);
  PSENscanInternalAngle expected_angle_end(2047);
  Degree expected_x_axis_rotation(x_axis_rotation);

  RosParameterHandler param_handler(node_handle);
  EXPECT_EQ(param_handler.getPassword(), expected_password_);
  EXPECT_EQ(param_handler.getSensorIP(), sensor_ip_);
  EXPECT_EQ(param_handler.getHostIP(), expected_host_ip_);
  EXPECT_EQ(param_handler.getHostUDPPort(), expected_host_udp_port_);
  EXPECT_EQ(param_handler.getFrameID(), frame_id);
  EXPECT_EQ(param_handler.getSkip(), expected_skip);
  EXPECT_EQ(param_handler.getAngleStart(), expected_angle_start);
  EXPECT_EQ(param_handler.getAngleEnd(), expected_angle_end);
  EXPECT_EQ(param_handler.getXAxisRotation(), expected_x_axis_rotation);
  EXPECT_EQ(param_handler.getPublishTopic(), publish_topic);
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_password)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set password with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("password", 15);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("frame_id", DEFAULT_FRAME_ID);
  ros::param::set("skip", DEFAULT_SKIP);
  ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
  ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
  ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  ros::param::set("publish_topic", DEFAULT_PUBLISH_TOPIC);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("password", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set password back to valid data type but not in accepted format
  ros::param::set("password", "AABBCCDDEEFFGG");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set password back to valid data type
  ros::param::set("password", password_);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_host_ip)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set host_ip with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("password", password_);
  ros::param::set("host_ip", 25);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("frame_id", DEFAULT_FRAME_ID);
  ros::param::set("skip", DEFAULT_SKIP);
  ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
  ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
  ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  ros::param::set("publish_topic", DEFAULT_PUBLISH_TOPIC);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("host_ip", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set host_ip back to valid data type but not IP-Format
  ros::param::set("host_ip", "AABBCCDDEEFFGG");
  // ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);
  ASSERT_ANY_THROW(RosParameterHandler param_handler(node_handle);
                   EXPECT_EQ(static_cast<uint32_t>(0), param_handler.getHostIP()));

  // test valid
  ros::param::set("host_ip", "1.2.3.4");
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_host_udp_port)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set host_udp_port with wrong datatype (expected int) as example for wrong datatypes on expected strings
  ros::param::set("password", password_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", true);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("frame_id", DEFAULT_FRAME_ID);
  ros::param::set("skip", DEFAULT_SKIP);
  ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
  ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
  ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  ros::param::set("publish_topic", DEFAULT_PUBLISH_TOPIC);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Wrong Datatype, but could be converted to int
  ros::param::set("host_udp_port", "52425");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Wrong Datatype, but could be converted to int, but it's too large
  ros::param::set("host_udp_port", "152425");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Wrong Datatype, but can't be converted to int
  ros::param::set("host_udp_port", "AABBCCDDEEFFGG");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Integer too large
  ros::param::set("host_udp_port", 150000);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set negative parameter
  ros::param::set("host_udp_port", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // test valid
  ros::param::set("host_udp_port", 1000);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_sensor_ip)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set sensor_ip with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("password", password_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("sensor_ip", 35);
  ros::param::set("frame_id", DEFAULT_FRAME_ID);
  ros::param::set("skip", DEFAULT_SKIP);
  ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
  ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
  ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  ros::param::set("publish_topic", DEFAULT_PUBLISH_TOPIC);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("sensor_ip", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set sensor_ip back to valid data type, but not valid IP-Address
  ros::param::set("sensor_ip", "AABBCCDDEEFFGG");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // test valid
  ros::param::set("sensor_ip", "1.2.3.4");
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_frame_id)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set frame_id with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("password", password_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("frame_id", 12);
  ros::param::set("skip", DEFAULT_SKIP);
  ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
  ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
  ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  ros::param::set("publish_topic", DEFAULT_PUBLISH_TOPIC);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("frame_id", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set frame_id back to valid data type, which could be interpreted as int
  // Numbers are allowed for frame id TODO: discussion
  ros::param::set("frame_id", "125");
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_skip)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set skip with wrong datatype (expected int) as example for wrong datatypes on expected strings
  ros::param::set("password", password_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("frame_id", DEFAULT_FRAME_ID);
  ros::param::set("skip", "abcde");
  ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
  ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
  ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  ros::param::set("publish_topic", DEFAULT_PUBLISH_TOPIC);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("skip", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Wrong Datatype, but could be converted to int
  ros::param::set("skip", "12");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set skip back to valid data type, but negative
  ros::param::set("skip", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // valid test
  ros::param::set("skip", 0);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_angle_start)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set angle_start with wrong datatype (expected int) as example for wrong datatypes on expected strings
  ros::param::set("password", password_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("frame_id", DEFAULT_FRAME_ID);
  ros::param::set("skip", DEFAULT_SKIP);
  ros::param::set("angle_start", "string");
  ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
  ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  ros::param::set("publish_topic", DEFAULT_PUBLISH_TOPIC);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("angle_start", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Wrong Datatype, but can be interpreted as int
  ros::param::set("angle_start", "12");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set angle_start back to valid data type, but too big
  ros::param::set("angle_start", 276);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set angle_start back to valid data type, but negative
  ros::param::set("angle_start", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // valid test
  ros::param::set("angle_start", 20.);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_angle_end)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set angle_end with wrong datatype (expected int) as example for wrong datatypes on expected strings
  ros::param::set("password", password_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("frame_id", DEFAULT_FRAME_ID);
  ros::param::set("skip", DEFAULT_SKIP);
  ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
  ros::param::set("angle_end", "string");
  ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  ros::param::set("publish_topic", DEFAULT_PUBLISH_TOPIC);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("angle_end", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Wrong Datatype, but can be converted to int
  ros::param::set("angle_end", "250");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set angle_end back to valid data type, but too large
  ros::param::set("angle_end", 276);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set angle_end back to valid data type, but negative
  ros::param::set("angle_end", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // valid test
  ros::param::set("angle_end", 90.);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_x_axis_rotation)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set x_axis_rotation with wrong datatype (expected float) as example for wrong datatypes on expected strings
  ros::param::set("password", password_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("frame_id", DEFAULT_FRAME_ID);
  ros::param::set("skip", DEFAULT_SKIP);
  ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
  ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
  ros::param::set("x_axis_rotation", "string");
  ros::param::set("publish_topic", DEFAULT_PUBLISH_TOPIC);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("x_axis_rotation", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Wrong Datatype, but can be converted to float
  ros::param::set("x_axis_rotation", "137.5");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set x_axis_rotation back to valid data type, but too large
  ros::param::set("x_axis_rotation", 361);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set x_axis_rotation back to valid data type, but too small
  ros::param::set("x_axis_rotation", -361);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // valid test
  ros::param::set("x_axis_rotation", 90.);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST_F(ROSParameterHandlerTest, test_invalid_params_publish_topic)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set all parameters to some value as a starting condition
  // Set publish_topic with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("password", password_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("frame_id", DEFAULT_FRAME_ID);
  ros::param::set("skip", DEFAULT_SKIP);
  ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
  ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
  ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  ros::param::set("publish_topic", 55);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("publish_topic", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set publish_topic back to valid data type, but it could be interpreted as int
  // accept this. TODO discussion
  ros::param::set("publish_topic", "55");
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle));
}

TEST(decryptPasswordTest, charOutOfRange)
{
  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBCCDDEEFFGG"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("GGAABBCCDDEEFF"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBCGGCDDEEFF"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBGACCDDEEFF"), DecryptPasswordException);

  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBCCDDEEFF@@"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("@@AABBCCDDEEFF"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("AABBCCDDEEFF@@"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("A@@ABBCCDDEEFF"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("AA@ABBCCDDEEFF"), DecryptPasswordException);

  EXPECT_THROW(RosParameterHandler::decryptPassword("aabbccddeeffgg"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("ggaabbccddeeff"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("``aabbccddeeff"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("aabbccddeeff``"), DecryptPasswordException);

  EXPECT_THROW(RosParameterHandler::decryptPassword("//345678901234"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("345678901234::"), DecryptPasswordException);
}

TEST(decryptPasswordTest, unevenCharacterCount)
{
  EXPECT_THROW(RosParameterHandler::decryptPassword("ac0d8d033"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("aC068d033"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("aC0d68D03"), DecryptPasswordException);
  EXPECT_THROW(RosParameterHandler::decryptPassword("ac0d6 8d03"), DecryptPasswordException);
  EXPECT_NO_THROW(RosParameterHandler::decryptPassword("ac0d68d033"));
  EXPECT_NO_THROW(RosParameterHandler::decryptPassword("ac0d6 8d033"));
}

TEST(decryptPasswordTest, controlCharactersBelow32)
{
  EXPECT_THROW(RosParameterHandler::decryptPassword("CD3195"), DecryptPasswordException);  //"\0\0\0"
  EXPECT_THROW(RosParameterHandler::decryptPassword("AA3111"), DecryptPasswordException);  // In der Mitte "\0"
  EXPECT_THROW(RosParameterHandler::decryptPassword("c4fd50ca29"),
               DecryptPasswordException);  // Tab am Anfang "\TABTEST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec8cca29"),
               DecryptPasswordException);  // Tab in der Mitte "TE\TABST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec66c554"),
               DecryptPasswordException);  // Tab am Ende "TEST\TAB)"
  EXPECT_THROW(RosParameterHandler::decryptPassword("d2fd50ca29"),
               DecryptPasswordException);  // \31 am Anfang "\31TEST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec9Aca29"),
               DecryptPasswordException);  // \31 in der Mitte "TE\31ST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec66c562"), DecryptPasswordException);  // Tab am Ende "TEST/31"
}

TEST(decryptPasswordTest, correctDecryption)
{
  EXPECT_EQ(RosParameterHandler::decryptPassword("8e0987d04eadfc68c380e74a"), "Christian123");
  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b115640c70d438"), "Giuseppe!!!!");
  EXPECT_EQ(RosParameterHandler::decryptPassword("9e1086da35a04aae1276da3e"), "Sascha??????");

  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b1 15640c70d438"), "Giuseppe!!!!");
  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b11 5640c70d438"), "Giuseppe!!!!");

  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b115640c70d438 "), "Giuseppe!!!!");
  EXPECT_EQ(RosParameterHandler::decryptPassword(" 8a0880ea38b115640c70d438"), "Giuseppe!!!!");

  EXPECT_EQ(RosParameterHandler::decryptPassword("  8 a0880e a38b115 640c7  0d4   38   "), "Giuseppe!!!!");
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_parameter_handler_test");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}