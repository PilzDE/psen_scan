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

#include "psen_scan/psen_scan_udp_interface.h"
#include "psen_scan/scanner_data.h"
#include <boost/chrono/chrono_io.hpp>
#include "psen_scan/udp_read_timeout_exception.h"
namespace psen_scan
{
/**
 * @brief Construct a new PSENscanUDPInterface::PSENscanUDPInterface object
 *
 * @param io_service Boost communication class
 * @param scanner_ip IP Adress if the scanner
 * @param host_udp_port UDP Port to receive the data from the scanner
 */

PSENscanUDPInterface::PSENscanUDPInterface(boost::asio::io_service& io_service, const std::string& scanner_ip, const uint32_t& host_udp_port)
:socket_(io_service, udp::endpoint(udp::v4(), host_udp_port)),
udp_endpoint_write_(boost::asio::ip::address_v4::from_string(scanner_ip), PSEN_SCAN_PORT)
{

}

/**
 * @brief Send commands to the scanner device.
 *
 * @param buffer Boost send buffer class.
 */

void PSENscanUDPInterface::write( const boost::asio::mutable_buffers_1& buffer )
{
  socket_.send_to(buffer, udp_endpoint_write_);
}

/**
 * @brief Receive data from the scanner.
 *
 * @param buffer Boost receive buffer class.
 * @return std::size_t Returns how many bytes have been read.
 *
 * @throws UDPReadTimeoutException
 */

std::size_t PSENscanUDPInterface::read( boost::asio::mutable_buffers_1& buffer )
{
  static int duration_counter=1;
  typedef boost::chrono::system_clock Clock;
  typedef boost::chrono::duration<long, boost::ratio<1>> Second;
  Clock::time_point t1 = Clock::now();
  Clock::duration d = Clock::now() - t1;
  while (0 == socket_.available())
  {
     d = Clock::now() - t1;
     Second s(duration_counter);
     if (d > s)
     {
       if (60 > duration_counter) duration_counter += 10;
       throw UDPReadTimeoutException("Could not receive UDP packet.");
     }
  };
  duration_counter = 1;
  return socket_.receive_from(buffer, udp_endpoint_read_);
}


/**
 * @brief Get the Udp Endpoint object for reading
 *
 * @return udp::endpoint
 */
udp::endpoint PSENscanUDPInterface::getUdpEndpointRead()
{
  return udp_endpoint_read_;
};

}