cmake_minimum_required(VERSION 3.8)
project(icar_middleware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(var_dependencies
    cv_bridge
    icar_interfaces
    pandu_ros2_kit
    pcl_conversions
    pcl_ros
    rclcpp
    std_srvs
    tf2
    tf2_ros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time filesystem)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(TensorRT REQUIRED)
foreach(dependency ${var_dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_transform src/lidar_transform.cpp)
add_executable(pose_estimator src/pose_estimator.cpp)
add_executable(px_cm_inference src/px_cm_inference.cpp)
add_executable(road_segmentation src/road_segmentation.cpp)
add_executable(transform_broadcaster src/transform_broadcaster.cpp)
ament_target_dependencies(lidar_transform ${var_dependencies})
ament_target_dependencies(pose_estimator ${var_dependencies})
ament_target_dependencies(
  px_cm_inference
  ${var_dependencies}
  Boost
  CUDA
  TensorRT)
ament_target_dependencies(road_segmentation ${var_dependencies})
ament_target_dependencies(transform_broadcaster ${var_dependencies})
target_link_libraries(lidar_transform ${PCL_LIBRARIES})
target_link_libraries(px_cm_inference nvonnxparser)
target_link_libraries(road_segmentation ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

install(
  TARGETS lidar_transform
          pose_estimator
          px_cm_inference
          road_segmentation
          transform_broadcaster
  DESTINATION lib/${PROJECT_NAME})

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
