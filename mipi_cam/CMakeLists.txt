cmake_minimum_required(VERSION 3.5)
project(mipi_cam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

OPTION(TROS    "TROS work environment"    ON)

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
option(IMAGE_TRANSPORT_PKG "using image_transport pkg" OFF)
if (${IMAGE_TRANSPORT_PKG})
  add_definitions(-DIMAGE_TRANSPORT_PKG_ENABLED)
  message("using image_transport pkg")
else()
  message("not using image_transport pkg")
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-w)
if(TROS)
add_definitions(-DUSING_HBMEM)
find_package(hbm_img_msgs REQUIRED)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if (${IMAGE_TRANSPORT_PKG})
  find_package(image_transport REQUIRED)
  find_package(image_transport_plugins REQUIRED)
  find_package(camera_info_manager REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio)
  message(STATUS "opencv version ${OpenCV_VERSION}")
endif ()

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# set(SYS_ROOT "")
message("SYS_ROOT is " ${SYS_ROOT})
include_directories(include
  src/x3
  src/x3_sdk_swap/include
  ${SYS_ROOT}/usr/include/vio/
)

if(TROS)
link_directories(
  ${SYS_ROOT}/usr/lib/hobot/
  ${SYS_ROOT}/usr/lib/hobot/hbmedia/
)
else()
link_directories(
  ${SYS_ROOT}/usr/lib/hobot/
  ${SYS_ROOT}/usr/lib/hobot/hbmedia/
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

## Declare a cpp executable
# ament_auto_add_library(${PROJECT_NAME}_node SHARED
#   src/mipi_cam_node.cpp
# )
# set(BASE_LIBRARIES pthread vio cam uvc dl)
set(BASE_LIBRARIES pthread vio cam dl)
# set(X3_HAPI_LIB guvc isp_algo  z avformat hbmedia turbojpeg isp iar tinyalsa avcodec avutil ion multimedia cjson diag gdcbin)
set(X3_HAPI_LIB isp_algo z hbmedia turbojpeg isp ion multimedia iar tinyalsa cjson diag gdcbin)

target_link_libraries(${PROJECT_NAME}
  ${BASE_LIBRARIES} ${X3_HAPI_LIB}
  ${ROS_LIB}
  ${OpenCV_LIBRARIES}
  # ${avcodec_LIBRARIES}
  # ${swscale_LIBRARIES}
  #guvc
  # TODO(oal) should this have been in libavcodec?
  #avutil
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
  )

if(TROS)
target_link_libraries(${PROJECT_NAME}
  hbmem
)
endif()

if (${IMAGE_TRANSPORT_PKG})
  ament_target_dependencies(
    ${PROJECT_NAME}
    cv_bridge
    image_transport
    camera_info_manager
    image_transport_plugins
    )
endif()

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
