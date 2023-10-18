#include "icar_routine/routine.hpp"

class Routine : public rclcpp::Node {
 public:
  Routine() : Node("routine") {}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_routine = std::make_shared<Routine>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_routine);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}