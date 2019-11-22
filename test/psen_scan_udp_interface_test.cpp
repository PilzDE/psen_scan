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

#include <psen_scan/psen_scan_udp_interface.h>
#include <gtest/gtest.h>
#include "mock_udp_server.h"
#include <boost/thread.hpp>
#include "psen_scan/udp_read_timeout_exception.h"
#include "psen_scan/fetch_monitoring_frame_exception.h"

using namespace psen_scan;

namespace psen_scan_test
{

TEST(PSENscanUDPInterfaceTest, test_all)
{
  boost::asio::io_service io_service_1, io_service_2;
  uint32_t mock_udp_port = 3000;
  uint32_t udp_port = 55000;
  std::string ip_addr = "127.0.0.1";
  udp::endpoint mock_endpoint(boost::asio::ip::address_v4::from_string(ip_addr), mock_udp_port);

  MockUDPServer mock_udp_server(io_service_1, mock_udp_port);

  boost::thread thrd( boost::bind( &boost::asio::io_service::run, &io_service_1 ) );

  std::unique_ptr<PSENscanUDPInterface> udp_interface = std::unique_ptr<PSENscanUDPInterface>
                                                        (
                                                          new PSENscanUDPInterface(io_service_2, ip_addr, udp_port)
                                                        );

  boost::array<char, 10> write_buf = {"Hello!"};
  boost::array<char, 10> read_buf;
  udp_interface->write(boost::asio::buffer(write_buf));

  auto temp = boost::asio::buffer(read_buf);
  unsigned int counter = 0;
  do
  {
    ASSERT_EQ(sizeof(write_buf), udp_interface->read(temp));
    counter++;
  }while((read_buf != write_buf) && (counter < 10));

  EXPECT_EQ(read_buf, write_buf);
  EXPECT_EQ(mock_endpoint, udp_interface->getUdpEndpointRead());

  mock_udp_server.block=1;
  udp_interface->write(boost::asio::buffer(write_buf));

  EXPECT_THROW( udp_interface->read(temp),UDPReadTimeoutException);

}

}