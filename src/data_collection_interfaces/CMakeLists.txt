cmake_minimum_required(VERSION 3.8)
project(data_collection_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RecordingFrame.msg"
  "srv/SessionCommand.srv"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
