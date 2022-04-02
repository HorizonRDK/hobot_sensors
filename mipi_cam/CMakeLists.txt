cmake_minimum_required(VERSION 3.5)
project(mipi_cam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

OPTION(SHARED_MEM    "SHARED_MEM work environment"    OFF)

# Using image_transport pkg to publish compressed img with default config
# 
# Publishing nv12 img only and do not depend on image_transport pkg:
# colcon build --packages-select mipi_cam --cmake-args -DIMAGE_TRANSPORT_PKG=OFF
# 
# For tros, the building cmd is:
# colcon build --packages-select mipi_cam \
#   --merge-install \
#   --cmake-force-configure \
#   --cmake-args \
#   --no-warn-unused-cli \
#   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake \
#   -DIMAGE_TRANSPORT_PKG=OFF \
#   -DSYS_ROOT=/mnt/test/cc_ws/sysroot_docker/

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-w)
if(SHARED_MEM)
add_definitions(-DUSING_HBMEM)
find_package(hbm_img_msgs REQUIRED)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# set(SYS_ROOT "")
message("SYS_ROOT is " ${SYS_ROOT})
include_directories(include
  src/x3
  src/x3_sdk_swap/include
  ${SYS_ROOT}/usr/include/vio/
)

if(SHARED_MEM)
link_directories(
  ${SYS_ROOT}/usr/lib/
  ${SYS_ROOT}/usr/lib/hbmedia/
)
else()
link_directories(
  ${SYS_ROOT}/usr/lib/
  ${SYS_ROOT}/usr/lib/hbmedia/
  /opt/ros/foxy/lib
)
endif()

## Build the mipi camera library
add_executable(${PROJECT_NAME}
  src/camera.cpp
  src/mipi_cam_node.cpp
  src/mipi_cam.cpp
  src/mipi_cap.cpp
#  src/x3/*.c
  src/x3/sensor_f37_config.c
  src/x3/sensor_imx415_config.c
  src/x3/x3_preparam.c
  src/x3/x3_utils.c
  src/x3_sdk_swap/src/x3_sdk_wrap.c
  src/x3_sdk_swap/src/x3_vio_bind.c
  src/x3_sdk_swap/src/x3_vio_rgn.c
  src/x3_sdk_swap/src/x3_vio_utils.c
  src/x3_sdk_swap/src/x3_vio_vdec.c
  src/x3_sdk_swap/src/x3_vio_vin.c
  src/x3_sdk_swap/src/x3_vio_vot.c
  src/x3_sdk_swap/src/x3_vio_vps.c
  src/x3_sdk_swap/src/x3_vio_venc.cpp
  src/x3_sdk_swap/src/x3_vio_vp.cpp
)

set(BASE_LIBRARIES pthread vio cam dl)
set(X3_HAPI_LIB isp_algo z hbmedia turbojpeg isp ion multimedia iar tinyalsa cjson diag gdcbin)

target_link_libraries(${PROJECT_NAME}
  ${BASE_LIBRARIES} ${X3_HAPI_LIB}
  ${ROS_LIB}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
  )

if(SHARED_MEM)
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

ament_package()
