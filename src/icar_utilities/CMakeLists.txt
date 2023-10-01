cmake_minimum_required(VERSION 3.8)
project(icar_utilities)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(icar_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_dateset_labeler src/camera_dataset_labeler.cpp)
ament_target_dependencies(
  camera_dateset_labeler
  rclcpp
  sensor_msgs
  cv_bridge
  tf2
  OpenCV)

add_executable(pixel_cm_visualizer src/pixel_cm_visualizer.cpp)
ament_target_dependencies(
  pixel_cm_visualizer
  rclcpp
  sensor_msgs
  cv_bridge
  tf2
  icar_interfaces
  OpenCV)

install(TARGETS camera_dateset_labeler DESTINATION lib/${PROJECT_NAME})
install(TARGETS pixel_cm_visualizer DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
