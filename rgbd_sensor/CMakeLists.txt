cmake_minimum_required(VERSION 3.5)
project(rgbd_sensor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

OPTION(BUILD_HBMEM    "BUILD_HBMEM work environment"    ON)
OPTION(BORAD_UBUNTU    "Board is ubuntu"    ON)

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

add_definitions(-std=c++11 -DRGBD -DHORIZON_PLATFORM)

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include
  src/TOF_SDK/include
  src/application
  src/application/camera
  src/application/common
  src/application/i2c
  src/application/tof_depth
  src/application/v4l2_capture
)

include_directories(
  ${CMAKE_SYSROOT}/usr/include/vio
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbmedia/
  src/TOF_SDK/libs
)

if(BORAD_UBUNTU)
include_directories(
  src/application/horizon_mpp/
)
else()
include_directories(
  src/application/horizon_mpp
  src/application/horizon_mpp/linaro/include
)
link_directories(
  src/application/horizon_mpp/linaro/libs
)
endif()

## Build the mipi camera library
add_executable(${PROJECT_NAME}
  src/rgbd_sensor.cpp
  src/rgbd_node.cpp
  src/rgbd_cam.cpp
  src/sunny_camera.cpp
  
  src/application/camera/camera_control.cpp
  src/application/camera/camera_hb_cfg.cpp
  
  src/application/horizon_mpp/horizon_mpp.cpp
  src/application/i2c/tof_i2c.cpp
  src/application/tof_depth/tof_depth_process.cpp
)

set(BASE_LIBRARIES dl rt pthread tof_mod_sdk tof_rgbd_sdk)
set(X3_HAPI_LIB hbmedia isp vio alog cam ion gdcbin hbmem iar tinyalsa)

target_link_libraries(${PROJECT_NAME}
  ${BASE_LIBRARIES} ${X3_HAPI_LIB}
  # ${ROS_LIB}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
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
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/TOF_SDK/libs/libtof_mod_sdk.so
  ${CMAKE_CURRENT_SOURCE_DIR}/src/TOF_SDK/libs/libtof_rgbd_sdk.so
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensorlib/libgc2053.so
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensorlib/libgc2053_linear.so
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensorlib/libirs2381c.so
  DESTINATION lib/)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/src/TOF_SDK/parameter
  DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
