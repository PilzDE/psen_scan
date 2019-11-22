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

#ifndef PSEN_SCAN_TEST_MOCK_UDP_SERVER_H
#define PSEN_SCAN_TEST_MOCK_UDP_SERVER_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

using boost::asio::ip::udp;

namespace psen_scan_test
{

/**
 * @brief Class for the UDP communication with the scanner.
 *
 */
class MockUDPServer
{
  public:
    int block;
    MockUDPServer(boost::asio::io_service& io_service,
                        const uint32_t& host_udp_port)
    :socket_(io_service, udp::endpoint(udp::v4(), host_udp_port))
    {
      block=0;
      start_receive();
    }

  private:
    void start_receive()
    {
      socket_.async_receive_from(
          boost::asio::buffer(recv_buffer_), remote_endpoint_,
          boost::bind(&MockUDPServer::handle_receive, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }

    void handle_receive(const boost::system::error_code& error,
        std::size_t /*bytes_transferred*/)
    {
      if (!error || error == boost::asio::error::message_size)
      {
        if (0 == block)
        {
        socket_.async_send_to(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            boost::bind(&MockUDPServer::handle_send, this,
              boost::asio::placeholders::error,
              boost::asio::placeholders::bytes_transferred));
        }
        start_receive();
      }
    }

    void handle_send(const boost::system::error_code& /*error*/,
        std::size_t /*bytes_transferred*/)
    {
    }

    udp::socket socket_; /**< Socket used for communication with Laserscanner. */
    udp::endpoint remote_endpoint_;
    boost::array<char, 100> recv_buffer_;
};

}

#endif // PSEN_SCAN_TEST_MOCK_UDP_SERVER_H
