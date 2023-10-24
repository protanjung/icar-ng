cmake_minimum_required(VERSION 3.8)
project(icar_io)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "/usr/local/share/cmake/GeographicLib/")

set(var_dependencies
    rclcpp
    pcl_conversions
    std_msgs
    sensor_msgs
    geometry_msgs
    icar_interfaces
    lwgps
    pandu_ros2_kit)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(PCL REQUIRED)
foreach(vardependency ${var_dependencies})
  find_package(${vardependency} REQUIRED)
endforeach()

include_directories(${GeographicLib_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(io_stm32 src/io_stm32.cpp)
add_executable(io_gps src/io_gps.cpp)
add_executable(io_lslidar_c16 src/io_lslidar_c16.cpp)
ament_target_dependencies(io_stm32 ${var_dependencies})
ament_target_dependencies(io_gps ${var_dependencies})
ament_target_dependencies(io_lslidar_c16 ${var_dependencies})
target_link_libraries(io_gps ${GeographicLib_LIBRARIES} serialport)
target_link_libraries(io_lslidar_c16 ${PCL_LIBRARIES})

install(TARGETS io_stm32 io_gps io_lslidar_c16 DESTINATION lib/${PROJECT_NAME})

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
