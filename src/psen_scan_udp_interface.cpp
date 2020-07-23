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

#include "psen_scan/psen_scan_udp_interface.h"
#include "psen_scan/udp_read_timeout_exception.h"
#include "psen_scan/scanner_data.h"
#include <boost/chrono/chrono_io.hpp>
#include <chrono>
#include <thread>

using boost::asio::ip::udp;

namespace psen_scan
{
static const uint64_t TIMEOUT_LOOP_SLEEP_DURATION_MS = 5;

/**
 * @brief Construct a new PSENscanUDPInterface::PSENscanUDPInterface object
 *
 * @param io_service Boost communication class
 * @param scanner_ip IP Adress if the scanner
 * @param host_udp_port UDP Port to receive the data from the scanner
 */

PSENscanUDPInterface::PSENscanUDPInterface(const std::string& scanner_ip, const uint32_t& host_udp_port)
  : socket_write_(io_service_, udp::endpoint(udp::v4(), host_udp_port + 1))
  , socket_read_(io_service_, udp::endpoint(udp::v4(), host_udp_port))
  , udp_write_endpoint_(boost::asio::ip::address_v4::from_string(scanner_ip), PSEN_SCAN_PORT_WRITE)
  , udp_read_endpoint_(boost::asio::ip::address_v4::from_string(scanner_ip), PSEN_SCAN_PORT_READ)
{
}

PSENscanUDPInterface::~PSENscanUDPInterface()
{
  close();
}

void PSENscanUDPInterface::open()
{
  try
  {
    socket_write_.connect(udp_write_endpoint_);
    socket_read_.connect(udp_read_endpoint_);
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw ScannerOpenFailed(ex.what());
  }
  // LCOV_EXCL_STOP
}

void PSENscanUDPInterface::close()
{
  try
  {
    socket_write_.close();
    socket_read_.close();
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw ScannerCloseFailed(ex.what());
  }
  // LCOV_EXCL_STOP
}

/**
 * @brief Send commands to the scanner device.
 *
 * @param buffer Boost send buffer class.
 */

void PSENscanUDPInterface::write(const boost::asio::mutable_buffers_1& buffer)
{
  try
  {
    socket_write_.send(buffer);
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw ScannerWriteFailed(ex.what());
  }
  // LCOV_EXCL_STOP
}

/**
 * @brief Receive data from the scanner.
 *
 * @param buffer Boost receive buffer class.
 * @return std::size_t Returns how many bytes have been read.
 *
 * @throws ScannerReadTimeout
 */

std::size_t PSENscanUDPInterface::read(boost::asio::mutable_buffers_1& buffer)
{
  static int duration_counter = 1;
  typedef boost::chrono::system_clock Clock;
  typedef boost::chrono::duration<int64_t, boost::ratio<1>> Second;
  Clock::time_point t1 = Clock::now();
  Clock::duration d = Clock::now() - t1;
  while (0 == socket_read_.available())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(TIMEOUT_LOOP_SLEEP_DURATION_MS));
    d = Clock::now() - t1;
    Second s(duration_counter);
    if (d > s)
    {
      if (60 > duration_counter)
      {
        duration_counter += 10;
      }
      throw ScannerReadTimeout();
    }
  };
  duration_counter = 1;

  std::size_t bytes_read{ 0 };
  try
  {
    bytes_read = socket_read_.receive(buffer);
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw ScannerReadFailed(ex.what());
  }
  // LCOV_EXCL_STOP

  return bytes_read;
}

}  // namespace psen_scan
