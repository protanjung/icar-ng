#ifndef ROUTINE_HPP_
#define ROUTINE_HPP_

#include "pandu_ros2_kit/help_marker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class Routine : public rclcpp::Node {
 public:
  //-----Parameter
  int icar_tyre_width;
  int icar_tyre_aspect_ratio;
  int icar_tyre_rim;
  float icar_body_length;
  float icar_body_width;
  float icar_body_height;
  float icar_wheelbase;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_10hz;
  rclcpp::TimerBase::SharedPtr tim_50hz;
  //-----Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  //-----Help
  HelpMarker _marker;

  // Inertial Measurement Unit
  // =========================
  double imu_roll, imu_pitch, imu_yaw;

  Routine();

  void cllbck_tim_10hz();
  void cllbck_tim_50hz();

  void cllbck_sub_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

  void process_metric();
  void process_marker();

  geometry_msgs::msg::Quaternion rpy_to_quaternion(float roll, float pitch, float yaw);
};

#endif  // ROUTINE_HPP_