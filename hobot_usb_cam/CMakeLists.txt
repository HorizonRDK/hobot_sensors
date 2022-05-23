cmake_minimum_required(VERSION 3.5)
project(hobot_usb_cam)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)
# find_package(v4l-utils)

include_directories(include/hobot_usb_cam)

add_executable(${PROJECT_NAME}
  src/hobot_usb_cam_exec.cpp
  src/hobot_usb_cam_node.cpp
  src/hobot_usb_cam.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs hbm_img_msgs)
# target_include_directories((${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>))

# target_link_libraries(${PROJECT_NAME}_node

#   ${avcodec_LIBRARIES}
#   ${swscale_LIBRARIES}
#   # TODO(lucasw) should this have been in libavcodec?
#   #avutil
# )

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()