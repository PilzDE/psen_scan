// Copyright (c) 2020 Pilz GmbH & Co. KG
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

#ifndef SCANNER_COMMUNICATION_INTERFACE_H
#define SCANNER_COMMUNICATION_INTERFACE_H

#include <stdexcept>

#include <boost/asio.hpp>

namespace psen_scan
{
class ScannerOpenFailed : public std::runtime_error
{
public:
  ScannerOpenFailed(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

class ScannerWriteFailed : public std::runtime_error
{
public:
  ScannerWriteFailed(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

class ScannerReadTimeout : public std::runtime_error
{
public:
  ScannerReadTimeout() : std::runtime_error("Timeout while waiting for message from scanner")
  {
  }
};

class ScannerReadFailed : public std::runtime_error
{
public:
  ScannerReadFailed(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

class ScannerCloseFailed : public std::runtime_error
{
public:
  ScannerCloseFailed(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

// LCOV_EXCL_START
/**
 * @brief Abstract base class for the communication interface with the PSENscan scanner.
 */
class ScannerCommunicationInterface
{
public:
  virtual ~ScannerCommunicationInterface() = default;

public:
  virtual void open() = 0;
  virtual void close() = 0;

  virtual void write(const boost::asio::mutable_buffers_1& buffer) = 0;
  virtual std::size_t read(boost::asio::mutable_buffers_1& buffer) = 0;
};
// LCOV_EXCL_STOP

}  // namespace psen_scan

#endif  // SCANNER_COMMUNICATION_INTERFACE_H
