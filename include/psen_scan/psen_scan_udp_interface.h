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

#ifndef PSEN_SCAN_UDP_INTERFACE_H
#define PSEN_SCAN_UDP_INTERFACE_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>

using boost::asio::ip::udp;

namespace psen_scan
{
// LCOV_EXCL_START
class UDPInterface
{
public:
  virtual ~UDPInterface() = default;
  virtual void write(const boost::asio::mutable_buffers_1& buffer) = 0;
  virtual std::size_t read(boost::asio::mutable_buffers_1& buffer) = 0;
};
// LCOV_EXCL_STOP

/**
 * @brief Class for the UDP communication with the scanner.
 *
 */
class PSENscanUDPInterface : public UDPInterface
{
public:
  PSENscanUDPInterface(boost::asio::io_service& io_service,
                       const std::string& scanner_ip,
                       const uint32_t& host_udp_port);
  void write(const boost::asio::mutable_buffers_1& buffer);
  std::size_t read(boost::asio::mutable_buffers_1& buffer);
  udp::endpoint getUdpWriteEndpoint() const;
  udp::endpoint getUdpReadEndpoint() const;

private:
  udp::socket socket_write_;         /**< Socket used for writing to Laserscanner. */
  udp::socket socket_read_;          /**< Socket used for reading from Laserscanner. */
  udp::endpoint udp_write_endpoint_; /**< Endpoint for writing to Laserscanner. */
  udp::endpoint udp_read_endpoint_;  /**< Endpoint for reading from Laserscanner. */
};
}

#endif  // PSEN_SCAN_UDP_INTERFACE_H