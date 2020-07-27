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

#ifndef PSEN_SCAN_TEST_MOCK_PSEN_SCAN_UDP_INTERFACE_H
#define PSEN_SCAN_TEST_MOCK_PSEN_SCAN_UDP_INTERFACE_H

#include <boost/asio.hpp>

#include <gmock/gmock.h>

#include <psen_scan/scanner_communication_interface.h>

namespace psen_scan_test
{
class MockPSENscanUDPInterface : public psen_scan::ScannerCommunicationInterface
{
public:
  MOCK_METHOD0(open, void());
  MOCK_METHOD0(close, void());

  MOCK_METHOD1(write, void(const boost::asio::mutable_buffers_1& buffer));
  MOCK_METHOD2(read, std::size_t(boost::asio::mutable_buffers_1& buffer, const std::chrono::steady_clock::duration));

private:
  boost::asio::ip::udp::udp::endpoint udp_endpoint_read_;
};
}  // namespace psen_scan_test

#endif  // PSEN_SCAN_TEST_MOCK_PSEN_SCAN_UDP_INTERFACE_H
