#include "icar_routine/routine.hpp"

Routine::Routine() : Node("routine") {
  //-----Parameter
  this->declare_parameter("icar.tyre.width", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("icar.tyre.aspect_ratio", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("icar.tyre.rim_diameter", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("icar.body.length", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("icar.body.width", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("icar.body.height", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("icar.wheelbase", rclcpp::PARAMETER_DOUBLE);
  this->get_parameter("icar.tyre.width", icar_tyre_width);
  this->get_parameter("icar.tyre.aspect_ratio", icar_tyre_aspect_ratio);
  this->get_parameter("icar.tyre.rim_diameter", icar_tyre_rim);
  this->get_parameter("icar.body.length", icar_body_length);
  this->get_parameter("icar.body.width", icar_body_width);
  this->get_parameter("icar.body.height", icar_body_height);
  this->get_parameter("icar.wheelbase", icar_wheelbase);
  //-----Timer
  tim_10hz = this->create_wall_timer(100ms, std::bind(&Routine::cllbck_tim_10hz, this));
  tim_50hz = this->create_wall_timer(20ms, std::bind(&Routine::cllbck_tim_50hz, this));
}

//====================================

void Routine::cllbck_tim_10hz() {
  if (!_marker.is_initialized()) { _marker.init(this->shared_from_this()); }

  process_metric();
  process_marker();
}

void Routine::cllbck_tim_50hz() {}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_routine = std::make_shared<Routine>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_routine);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}