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

#ifndef PSEN_SCAN_ROS_PARAMETER_HANDLER_H
#define PSEN_SCAN_ROS_PARAMETER_HANDLER_H


#include <psen_scan/scanner_data.h>
#include <ros/ros.h>
#include <psen_scan/default_parameters.h>

namespace psen_scan
{

const std::map<std::string, bool> ROS_PARAMETER =
                                                  {
                                                    {"password",      true},
                                                    {"host_ip",       true},
                                                    {"host_udp_port", true},
                                                    {"sensor_ip",     true},
                                                    {"frame_id",      false},
                                                    {"skip",          false},
                                                    {"angle_start",   false},
                                                    {"angle_end",     false},
                                                    {"x_axis_rotation", false},
                                                    {"publish_topic", false}
                                                  };

/**
 * @brief Class for getting ROS-Parameters from the parameter-server
 *
 */
class RosParameterHandler
{
  public:
    RosParameterHandler(const ros::NodeHandle& nh);
    std::string           getPassword() const;
    uint32_t                getHostIP() const;
    uint32_t           getHostUDPPort() const;
    std::string           getSensorIP() const;
    std::string            getFrameID() const;
    uint16_t                  getSkip() const;
    uint16_t            getAngleStart() const;
    uint16_t              getAngleEnd() const;
    double           getXAxisRotation() const;
    std::string       getPublishTopic() const;
  private:
    ros::NodeHandle       const nh_; /**< Nodehandle through which parameters are fetched */
    std::string           password_; /**< Password for Laserscanner */
    uint32_t               host_ip_; /**< IP-Adress of host machine */
    uint32_t         host_udp_port_; /**< UDP Port on which packets from Laserscanner should be received */
    std::string          sensor_ip_; /**< IP-Adress of Safety laser scanner */
    std::string           frame_id_; /**< ROS Frame ID */
    uint16_t                  skip_; /**< How many incoming frames should be skipped (reduces publish rate) */
    uint16_t           angle_start_; /**< Start angle of measurement */
    uint16_t             angle_end_; /**< End angle of measurement */
    double         x_axis_rotation_; /**< Rotation of x-axis arround the center */
    std::string      publish_topic_; /**< Topic to publish Laserscan data on */
    template<class T>
    T getParamFromNh(const ros::NodeHandle& nh, const std::string& key) const;

  public:
    static std::string decryptPassword(const std::string& encrypted_password);
};

}

#endif // PSEN_SCAN_ROS_PARAMETER_HANDLER_H