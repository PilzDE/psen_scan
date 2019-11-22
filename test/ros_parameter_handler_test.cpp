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
#include <psen_scan/decrypt_password_exception.h>
#include <psen_scan/psen_scan_fatal_exception.h>
#include <arpa/inet.h>
#include <gtest/gtest.h>

using namespace psen_scan;

#define DELETE_ROS_PARAM(param_name) \
          if( ros::param::has(param_name) ) \
          { \
            ros::param::del(param_name); \
          }

#define DELETE_ALL_ROS_PARAMS() \
          DELETE_ROS_PARAM("password"); \
          DELETE_ROS_PARAM("sensor_ip"); \
          DELETE_ROS_PARAM("host_ip"); \
          DELETE_ROS_PARAM("host_udp_port"); \
          DELETE_ROS_PARAM("angle_start"); \
          DELETE_ROS_PARAM("angle_end"); \
          DELETE_ROS_PARAM("frame_id"); \
          DELETE_ROS_PARAM("skip"); \
          DELETE_ROS_PARAM("publish_topic");

namespace psen_scan_test
{

class ROSParameterHandlerTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
      // Default values to set
      password_           = "ac0d68d033";
      sensor_ip_          =    "1.2.3.4";
      host_ip_            =    "1.2.3.5";
      host_udp_port_      =        12345;

      // Default expected values
      default_password_       =   "admin";
      default_frame_id_       = "scanner";
      default_skip_           =         0;
      default_publish_topic_  =    "scan";
      default_angle_start_    =         0;
      default_angle_end_      =      2750;

      expected_host_ip_        = htobe32(inet_network(host_ip_.c_str()));
      expected_host_udp_port_  =                 htole32(host_udp_port_);
    }

    // Default values to set
    std::string  password_;
    std::string sensor_ip_;
    std::string   host_ip_;
    int     host_udp_port_;

    // Default expected values
    std::string      default_password_;
    std::string      default_frame_id_;
    uint16_t             default_skip_;
    std::string default_publish_topic_;
    uint32_t         expected_host_ip_;
    uint32_t   expected_host_udp_port_;
    uint16_t      default_angle_start_;
    uint16_t        default_angle_end_;
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

  ASSERT_NO_THROW
  (
    RosParameterHandler param_handler(node_handle);
    EXPECT_EQ(param_handler.getPassword(), default_password_);
    EXPECT_EQ(param_handler.getSensorIP(), sensor_ip_);
    EXPECT_EQ(param_handler.getHostIP(), expected_host_ip_);
    EXPECT_EQ(param_handler.getHostUDPPort(), expected_host_udp_port_);
    EXPECT_EQ(param_handler.getFrameID(), default_frame_id_);
    EXPECT_EQ(param_handler.getSkip(), default_skip_);
    EXPECT_EQ(param_handler.getAngleStart(), default_angle_start_);
    EXPECT_EQ(param_handler.getAngleEnd(), default_angle_end_);
    EXPECT_EQ(param_handler.getPublishTopic(), default_publish_topic_);
  );

}

TEST_F(ROSParameterHandlerTest, test_single_required_params_missing)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  //ros::param::set("password", password_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("password", password_);
  DELETE_ROS_PARAM("sensor_ip");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("sensor_ip", sensor_ip_);
  DELETE_ROS_PARAM("host_ip");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("host_ip", host_ip_);
  DELETE_ROS_PARAM("host_udp_port");

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

  ros::param::set("password", password_);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);
  ros::param::set("angle_start", angle_start);
  ros::param::set("angle_end", angle_end);
  ros::param::set("frame_id", frame_id);
  ros::param::set("skip", skip);
  ros::param::set("publish_topic", publish_topic);

  uint16_t expected_skip = 2;
  uint16_t expected_angle_start = 105;
  uint16_t expected_angle_end = 2047;

  ASSERT_NO_THROW
  (
    RosParameterHandler param_handler(node_handle);
    EXPECT_EQ(param_handler.getPassword(), default_password_);
    EXPECT_EQ(param_handler.getSensorIP(), sensor_ip_);
    EXPECT_EQ(param_handler.getHostIP(), expected_host_ip_);
    EXPECT_EQ(param_handler.getHostUDPPort(), expected_host_udp_port_);
    EXPECT_EQ(param_handler.getFrameID(), frame_id);
    EXPECT_EQ(param_handler.getSkip(), expected_skip);
    EXPECT_EQ(param_handler.getAngleStart(), expected_angle_start);
    EXPECT_EQ(param_handler.getAngleEnd(), expected_angle_end);
    EXPECT_EQ(param_handler.getPublishTopic(), publish_topic);
  );

}

TEST_F(ROSParameterHandlerTest, test_invalid_params)
{
  ros::NodeHandle node_handle;

  DELETE_ALL_ROS_PARAMS();

  // Set password with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("password", 15);
  ros::param::set("sensor_ip", sensor_ip_);
  ros::param::set("host_ip", host_ip_);
  ros::param::set("host_udp_port", host_udp_port_);

  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("password", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  ros::param::set("password", "AABBCCDDEEFFGG");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

  // Set angle with wrong datatype (expected float) as example for wrong datatypes on expected floats
  ros::param::set("password", password_);
  ros::param::set("angle_start", true);
  ros::param::set("angle_end", "string");
  ASSERT_NO_THROW
    (
      RosParameterHandler param_handler(node_handle);
      EXPECT_EQ(param_handler.getAngleStart(), default_angle_start_);
      EXPECT_EQ(param_handler.getAngleEnd(), default_angle_end_);
    );

  ros::param::set("angle_start", 23);
  ros::param::set("angle_end", 83.8);
  ASSERT_NO_THROW
    (
      RosParameterHandler param_handler(node_handle);
      EXPECT_EQ(param_handler.getAngleStart(), 230);
      EXPECT_EQ(param_handler.getAngleEnd(), 838);
    );

  // Set skip with wrong datatype (expected int) as example for wrong datatypes on expected ints
  ros::param::set("skip", true);
  ASSERT_NO_THROW
    (
      RosParameterHandler param_handler(node_handle);
      EXPECT_EQ(param_handler.getSkip(), default_skip_);
    );

  ros::param::set("skip", "string");
  ASSERT_NO_THROW
    (
      RosParameterHandler param_handler(node_handle);
      EXPECT_EQ(param_handler.getSkip(), default_skip_);
    );

  ros::param::set("skip", 13.7);
  ASSERT_NO_THROW
    (
      RosParameterHandler param_handler(node_handle);
      EXPECT_EQ(param_handler.getSkip(), 14);
    );

  ros::param::set("skip", 13.5);
  ASSERT_NO_THROW
    (
      RosParameterHandler param_handler(node_handle);
      EXPECT_EQ(param_handler.getSkip(), 14);
    );

  ros::param::set("skip", 13.2);
  ASSERT_NO_THROW
    (
      RosParameterHandler param_handler(node_handle);
      EXPECT_EQ(param_handler.getSkip(), 13);
    );

  // Set negative parameters
  ros::param::set("host_udp_port", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);
  ros::param::set("host_udp_port", 0);

  ros::param::set("angle_start", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);
  ros::param::set("angle_start", 0);

  ros::param::set("angle_end", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);
  ros::param::set("angle_end", 0);

  ros::param::set("skip", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle), PSENScanFatalException);

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
  EXPECT_THROW(RosParameterHandler::decryptPassword("CD3195"),DecryptPasswordException); //"\0\0\0"
  EXPECT_THROW(RosParameterHandler::decryptPassword("AA3111"),DecryptPasswordException); //In der Mitte "\0"
  EXPECT_THROW(RosParameterHandler::decryptPassword("c4fd50ca29"),DecryptPasswordException); // Tab am Anfang "\TABTEST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec8cca29"),DecryptPasswordException); // Tab in der Mitte "TE\TABST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec66c554"),DecryptPasswordException); // Tab am Ende "TEST\TAB)"
  EXPECT_THROW(RosParameterHandler::decryptPassword("d2fd50ca29"),DecryptPasswordException); // \31 am Anfang "\31TEST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec9Aca29"),DecryptPasswordException); // \31 in der Mitte "TE\31ST"
  EXPECT_THROW(RosParameterHandler::decryptPassword("99ec66c562"),DecryptPasswordException); // Tab am Ende "TEST/31"
}

TEST(decryptPasswordTest, correctDecryption)
{
  EXPECT_EQ(RosParameterHandler::decryptPassword("8e0987d04eadfc68c380e74a"),"Christian123");
  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b115640c70d438"),"Giuseppe!!!!");
  EXPECT_EQ(RosParameterHandler::decryptPassword("9e1086da35a04aae1276da3e"),"Sascha??????");

  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b1 15640c70d438"),"Giuseppe!!!!");
  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b11 5640c70d438"),"Giuseppe!!!!");

  EXPECT_EQ(RosParameterHandler::decryptPassword("8a0880ea38b115640c70d438 "),"Giuseppe!!!!");
  EXPECT_EQ(RosParameterHandler::decryptPassword(" 8a0880ea38b115640c70d438"),"Giuseppe!!!!");

  EXPECT_EQ(RosParameterHandler::decryptPassword("  8 a0880e a38b115 640c7  0d4   38   "),"Giuseppe!!!!");
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_parameter_handler_test");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}