cmake_minimum_required(VERSION 3.8)
project(icar_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(var_dependencies std_msgs sensor_msgs geometry_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(dependency ${var_dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

set(msg_files
    msg/Dummy.msg
    msg/Stm32FromPc.msg
    msg/Stm32ToPc.msg
    msg/GpsToPc.msg
    msg/Pose.msg
    msg/LidarRing.msg
    msg/LidarRings.msg)
set(srv_files srv/Dummy.srv)
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  ${var_dependencies})
ament_export_dependencies(rosidl_default_runtime ${var_dependencies})

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
