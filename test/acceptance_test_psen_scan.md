<!--
Copyright © 2020 Pilz GmbH & Co. KG

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

-->

# Acceptance Test PSENscan using Rviz

## Prerequisites
  - Properly configure and start up the PSENscan safety laser scanner.

### Test Sequence

  1. Run
  ```
  roslaunch psen_scan psen_scan.launch sensor_ip:=<sensor_ip> host_ip:=<host_ip> host_udp_port:=<host_udp_port>
  ```
  Replace the variables in brackets with appropriate values for your setup.

### Expected Results
  1. Rviz shows a 2D pointcloud of distance values that match your surroundings.
