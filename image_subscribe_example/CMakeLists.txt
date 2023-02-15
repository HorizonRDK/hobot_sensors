# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(image_subscribe_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(BUILD_HBMEM)
add_definitions(-DUSING_HBMEM)
find_package(hbm_img_msgs REQUIRED)
add_definitions(-DBUILD_HBMEM_MSG)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include
${PROJECT_SOURCE_DIR}
)

if(BUILD_HBMEM)
link_directories(
  ${CMAKE_SYSROOT}/usr/lib/
)
endif()

add_executable(subscribe_example
src/example.cpp
src/image_subscriber.cpp
)

if(BUILD_HBMEM)
target_link_libraries(subscribe_example
  hbmem ion alog
)
ament_target_dependencies(
  subscribe_example
  rclcpp
  sensor_msgs
  hbm_img_msgs
)
else()
ament_target_dependencies(
  subscribe_example
  rclcpp
  sensor_msgs
)
endif()

target_link_libraries(subscribe_example
)

# Install executables
install(
  TARGETS subscribe_example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
