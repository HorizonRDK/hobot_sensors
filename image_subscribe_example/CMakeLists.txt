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

OPTION(SHARED_MEM    "SHARED_MEM work environment"    OFF)

if(SHARED_MEM)
add_definitions(-DUSING_HBMEM)
find_package(hbm_img_msgs REQUIRED)
add_definitions(-DSHARED_MEM_MSG)
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

if(SHARED_MEM)
link_directories(
  ${CMAKE_SYSROOT}/usr/lib/
)
endif()

add_executable(subscribe_example
src/example.cpp
src/image_subscriber.cpp
)

if(SHARED_MEM)
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
