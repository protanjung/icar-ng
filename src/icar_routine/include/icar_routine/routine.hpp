#ifndef ROUTINE_HPP_
#define ROUTINE_HPP_

#include "pandu_ros2_kit/help_marker.hpp"
#include "rclcpp/rclcpp.hpp"
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
  //-----Help
  HelpMarker _marker;

  Routine();

  void cllbck_tim_10hz();
  void cllbck_tim_50hz();

  void process_metric();
  void process_marker();

  geometry_msgs::msg::Quaternion quaternion_from_rpy(float roll, float pitch, float yaw);
};

#endif  // ROUTINE_HPP_