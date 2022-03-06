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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(avcodec libavcodec REQUIRED)
# pkg_check_modules(swscale libswscale REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(camera_info_manager REQUIRED)

find_package(cv_bridge REQUIRED)
#find_package(OpenCV 3.3 REQUIRED) #local 4.2
#find_package(OpenCV REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio)
message(STATUS "opencv version ${OpenCV_VERSION}")

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(include
  src/x3
  src/x3_sdk_swap/include
  # ${avcodec_INCLUDE_DIRS}
  # ${swscale_INCLUDE_DIRS}
  #/usr/include
  #/usr/include/vio
  #x3_prebuilt
  ./x3_prebuilt/include/appsdk/vio/
  ./x3_prebuilt/include/appsdk/
 
  /opt/ros/foxy/
  /opt/ros/foxy/include/
  /opt/ros/foxy/include/builtin_interfaces/
)
link_directories(
  #/usr/lib/hobot/
  #/usr/lib/hobot/hbmedia/
  ./x3_prebuilt/lib/appsdk/hbmedia/
  ./x3_prebuilt/lib/appsdk/
  /opt/ros/foxy/lib
)

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
set(BASE_LIBRARIES pthread vio cam uvc dl)
set(X3_HAPI_LIB guvc isp_algo  z avformat hbmedia turbojpeg isp iar tinyalsa avcodec avutil)

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
  #roscpp
  cv_bridge
  rclcpp
  sensor_msgs
  image_transport
  camera_info_manager
  image_transport_plugins
  )

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
