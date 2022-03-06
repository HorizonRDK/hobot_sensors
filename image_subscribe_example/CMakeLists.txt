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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include
${PROJECT_SOURCE_DIR}
)

set(IMAGE_SUBSCRIBE_INSTALL_PATH "../install/image_subscribe")
message("IMAGE_SUBSCRIBE_INSTALL_PATH: " ${IMAGE_SUBSCRIBE_INSTALL_PATH})
include_directories(
  ${IMAGE_SUBSCRIBE_INSTALL_PATH}/include
)

link_directories(
  ${IMAGE_SUBSCRIBE_INSTALL_PATH}/lib
)

add_executable(${PROJECT_NAME}
src/example.cpp
src/image_subscriber.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  cv_bridge
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
