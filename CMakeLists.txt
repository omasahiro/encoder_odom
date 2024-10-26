cmake_minimum_required(VERSION 3.8)
project(encoder_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(encoder_reader src/encoder_reader.cpp)
target_link_libraries(encoder_reader pigpiod_if2)

add_library(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher encoder_reader)
ament_target_dependencies(odom_publisher
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(encoder_odom_node src/main.cpp)
target_link_libraries(encoder_odom_node odom_publisher)
ament_target_dependencies(encoder_odom_node
  rclcpp
)

install(TARGETS
  encoder_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()