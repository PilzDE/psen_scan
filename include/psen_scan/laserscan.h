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

#ifndef PSEN_SCAN_LASERSCAN_H
#define PSEN_SCAN_LASERSCAN_H

#include <vector>
#include <psen_scan/scanner_frames.h>
#include <psen_scan/scanner_data.h>

namespace psen_scan {

/**
 * @brief Class to hold the data for one laserscan without depencies to ROS.
 *
 */
typedef struct LaserScan
{
  public:
    LaserScan(const uint8_t &resolution,
              const uint16_t &min_scan_angle,
              const uint16_t &max_scan_angle);

    std::vector<uint16_t> measures_; /**< Measurement data of the laserscan in Millimeters. */
    uint8_t  resolution_; /**< Distance of angle between the measurements in tenths of degree. */
    uint16_t const min_scan_angle_; /**< Lowest  Angle the Scanner is scanning in tenths of degree.*/
    uint16_t const max_scan_angle_; /**< Highest Angle the Scanner is scanning in tenths of degree.*/
} LaserScan;

}



#endif // PSEN_SCAN_LASERSCAN_H