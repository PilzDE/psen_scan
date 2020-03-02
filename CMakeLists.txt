# Copyright (c) 2019 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8.3)
project(psen_scan)

add_definitions(-std=c++11)
add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wno-unused-parameter)
add_definitions(-Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

###############
## Sanitizer ##
###############
# to run (only one sanitizer at a time):
# 1. Set environment variables for log_path in your shell
# export ASAN_OPTIONS="log_path=/path/to/asan-logs"
# export TSAN_OPTIONS="log_path=/path/to/tsan-logs"
# export UBSAN_OPTIONS="log_path=/path/to/ubsan-logs"
# 2. Compile
# catkin_make -DCATKIN_SANITIZER=address|thread|undefined
# 3. Run the node
# 4. Check log_path folder for contents
# build and devel folders have to be deleted before run
if(CATKIN_SANITIZER)
  set(SANITIZER_OPTIONS "address|thread|undefined")
  if(CATKIN_SANITIZER MATCHES ${SANITIZER_OPTIONS})
    message(STATUS "${CATKIN_SANITIZER}-sanitizer activated.")
    set(CMAKE_CXX_FLAGS "-fsanitize=${CATKIN_SANITIZER} -g")
  else()
    message(FATAL_ERROR "${CATKIN_SANITIZER} is not a valid sanitizer. Valid options are: [${SANITIZER_OPTIONS}]")
  endif()
endif()

################
## Clang tidy ##
################
# to run: catkin_make -DCATKIN_ENABLE_CLANG_TIDY=true
# build and devel folders have to be deleted before run
if(CATKIN_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_sources
  src/scanner.cpp
  src/scanner_frames.cpp
  src/laserscan.cpp
  src/psen_scan_internal_angle.cpp
  src/psen_scan_udp_interface.cpp
)

set(${PROJECT_NAME}_node_sources
  src/psen_scan_driver.cpp
  src/ros_parameter_handler.cpp
  src/ros_scanner_node.cpp
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_node_sources})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############
##  Test  ##
############
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(code_coverage REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    include(CodeCoverage)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  roslaunch_add_file_check(launch)

  include_directories(include ${catkin_INCLUDE_DIR})

  catkin_add_gmock(scanner_test
    test/scanner_test.cpp
    src/scanner.cpp
    src/scanner_frames.cpp
    src/laserscan.cpp
    src/psen_scan_internal_angle.cpp
    src/psen_scan_udp_interface.cpp
  )
  target_link_libraries(scanner_test
    ${catkin_LIBRARIES}
  )

  add_rostest_gmock(ros_scanner_node_test
    test/ros_scanner_node_test.test
    test/ros_scanner_node_test.cpp
    src/ros_scanner_node.cpp
    src/laserscan.cpp
    src/psen_scan_internal_angle.cpp
  )
  target_link_libraries(ros_scanner_node_test
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(psen_scan_internal_angle_test
    test/psen_scan_internal_angle_test.cpp
    src/psen_scan_internal_angle.cpp
  )
  target_link_libraries(psen_scan_internal_angle_test
    ${catkin_LIBRARIES}
    gtest_main
  )

  catkin_add_gtest(psen_scan_udp_interface_test
    test/psen_scan_udp_interface_test.cpp
    src/psen_scan_internal_angle.cpp
    src/psen_scan_udp_interface.cpp
  )
  target_link_libraries(psen_scan_udp_interface_test
    ${catkin_LIBRARIES}
    gtest_main
  )

  add_rostest_gtest(ros_parameter_handler_test
    test/ros_parameter_handler_test.test
    test/ros_parameter_handler_test.cpp
    src/ros_parameter_handler.cpp
    src/psen_scan_internal_angle.cpp
  )
  target_link_libraries(ros_parameter_handler_test
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(build_new_exceptions
    test/new_build_ros_message_exception_test.cpp
    test/new_coherent_monitoring_frames_exception_test.cpp
    test/new_decrypt_password_exception_test.cpp
    test/new_diagnostic_information_exception_test.cpp
    test/new_fetch_monitoring_frame_exception_test.cpp
    test/new_get_ros_parameter_exception_test.cpp
    test/new_parse_monitoring_frame_exception_test.cpp
    test/new_psen_scan_fatal_exception_test.cpp
    test/new_udp_read_timeout_exception_test.cpp
  )
  target_link_libraries(build_new_exceptions
    ${catkin_LIBRARIES}
    gtest_main
  )

  # to run: catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug psen_scan_coverage (adding -j1 recommended)
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*"
                          "*/${PROJECT_NAME}/src/psen_scan_driver.cpp")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()
endif()