cmake_minimum_required(VERSION 3.16)
project(camera_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
# Keep one editable configuration/calibration set in the checkout, including after install.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/source_directory" "${CMAKE_CURRENT_SOURCE_DIR}\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/source_directory" DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/init_gmsl.sh DESTINATION lib/${PROJECT_NAME})
ament_package()
