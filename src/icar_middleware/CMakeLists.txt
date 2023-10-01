cmake_minimum_required(VERSION 3.8)
project(icar_middleware)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic)
endif()

list(
  APPEND
  CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(
  APPEND
  CMAKE_PREFIX_PATH
  "${CMAKE_CURRENT_LIST_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(icar_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time filesystem)
find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS})

add_executable(pixel_cm_inference src/pixel_cm_inference.cpp)
ament_target_dependencies(
  pixel_cm_inference
  rclcpp
  icar_interfaces
  Boost
  CUDA
  TensorRT)
target_link_libraries(pixel_cm_inference nvonnxparser)

install(TARGETS pixel_cm_inference DESTINATION lib/${PROJECT_NAME})

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
