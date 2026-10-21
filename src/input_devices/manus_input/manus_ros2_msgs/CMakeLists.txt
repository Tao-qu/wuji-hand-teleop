cmake_minimum_required(VERSION 3.11)
project(manus_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Humble message generation uses the legacy Python variable.
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ManusGlove.msg
  msg/ManusRawNode.msg
  msg/ManusErgonomics.msg
  DEPENDENCIES geometry_msgs std_msgs)

ament_package()
