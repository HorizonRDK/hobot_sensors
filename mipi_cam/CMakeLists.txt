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
project(mipi_cam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-w)
if(BUILD_HBMEM)
add_definitions(-DUSING_HBMEM)
find_package(hbm_img_msgs REQUIRED)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(PREFIX_PATH x3)

message("PREFIX_PATH: ${PREFIX_PATH}")



if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
  set(SRC_PLATFORM_FILE 
    src/x3/x3_sdk_swap/src/x3_sdk_wrap.c
    src/x3/x3_sdk_swap/src/x3_vio_bind.c
    src/x3/x3_sdk_swap/src/x3_vio_vin.c
    src/x3/x3_sdk_swap/src/x3_vio_vps.c
    src/x3/x3_sdk_swap/src/x3_vio_vp.cpp
    src/x3/sensor/sensor_f37_config.c
    src/x3/sensor/sensor_gc4c33_config.c
    src/x3/sensor/sensor_gc4663_config.c
    src/x3/sensor/sensor_imx415_config.c
    src/x3/sensor/sensor_imx586_config.c
    src/x3/sensor/sensor_imx219_config.c
    src/x3/sensor/sensor_imx477_config.c
    src/x3/sensor/sensor_ov5647_config.c
    src/x3/sensor/x3_preparam.c
    src/x3/sensor/x3_utils.c
  )
  include_directories(include 
    src/x3/x3_sdk_swap/include
    src/x3/sensor
  )
elseif(PLATFORM_J5)
  message("build platform J5")
  add_definitions(-DPLATFORM_J5)
  set(PREFIX_PATH j5)
elseif(PLATFORM_X86)
  message("build platform X86")
  add_definitions(-DPLATFORM_X86)
  set(PREFIX_PATH x86)
  message("THIRD_PARTY is " ${THIRD_PARTY})
  include_directories(
    /usr/include/
    ${THIRD_PARTY}/usr/include/
  )
  link_directories(
    /lib/
    ${THIRD_PARTY}/usr/lib/hbbpu/
  )
else()
  message("invalid platform, build platform X3 default")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
endif()

include_directories(include
  include/${PREFIX_PATH}
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_SYSROOT}/usr/
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbmedia
)

add_executable(${PROJECT_NAME}
  src/camera.cpp
  src/hobot_mipi_node.cpp
  src/hobot_mipi_cam.cpp
  src/${PREFIX_PATH}/hobot_mipi_cap_iml.cpp
  src/${PREFIX_PATH}/hobot_mipi_factory.cpp
  ${SRC_PLATFORM_FILE}
)

set(BASE_LIBRARIES pthread vio cam dl)
if(PLATFORM_X3)
  set(X3_HAPI_LIB isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson diag gdcbin)

elseif(PLATFORM_J5)
elseif(PLATFORM_X86)
else()
  set(BASE_LIBRARIES pthread vio cam dl)
  set(X3_HAPI_LIB isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson diag gdcbin)
endif()


target_link_libraries(${PROJECT_NAME}
  ${BASE_LIBRARIES} ${X3_HAPI_LIB} yaml-cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
  yaml_cpp_vendor
  )

if(BUILD_HBMEM)
  target_link_libraries(${PROJECT_NAME}
  hbmem
)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  hbm_img_msgs
)
endif()

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
