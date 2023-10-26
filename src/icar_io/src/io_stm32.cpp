#include "icar_interfaces/msg/stm32_from_pc.hpp"
#include "icar_interfaces/msg/stm32_to_pc.hpp"
#include "pandu_ros2_kit/udp.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class IOSTM32 : public rclcpp::Node {
 public:
  //-----Parameter
  std::string stm32_ip;
  int stm32_port;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_50hz;
  //-----Subscriber
  rclcpp::Subscription<icar_interfaces::msg::Stm32FromPc>::SharedPtr sub_stm32_from_pc;
  //-----Publisher
  rclcpp::Publisher<icar_interfaces::msg::Stm32ToPc>::SharedPtr pub_stm32_to_pc;

  // Socket connection
  // =================
  UDP stm32_udp;
  uint8_t stm32_tx_buffer[2048];
  uint8_t stm32_rx_buffer[2048];

  // Data to PC
  // ==========
  uint32_t epoch_to_pc;
  uint16_t remote_control[16];
  uint16_t encoder_kiri;
  uint16_t encoder_kanan;
  float gyroscope;
  int16_t throttle_position;
  int16_t steering_position;

  // Data from PC
  // ============
  uint32_t epoch_from_pc;
  int16_t throttle;
  int16_t steering;
  uint8_t transmission;
  uint8_t accessory;

  IOSTM32() : Node("io_stm32") {
    //-----Parameter
    this->declare_parameter("stm32.ip", rclcpp::PARAMETER_STRING);
    this->declare_parameter("stm32.port", rclcpp::PARAMETER_INTEGER);
    this->get_parameter("stm32.ip", stm32_ip);
    this->get_parameter("stm32.port", stm32_port);
    //-----Timer
    tim_50hz = this->create_wall_timer(20ms, std::bind(&IOSTM32::cllbck_tim_50hz, this));
    //-----Subscriber
    sub_stm32_from_pc = this->create_subscription<icar_interfaces::msg::Stm32FromPc>(
        "stm32/from_pc", 10, std::bind(&IOSTM32::cllbck_sub_stm32_from_pc, this, std::placeholders::_1));
    //-----Publisher
    pub_stm32_to_pc = this->create_publisher<icar_interfaces::msg::Stm32ToPc>("stm32/to_pc", 10);

    if (stm32_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "STM32 init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_50hz() {
    if (stm32_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "STM32 routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_stm32_from_pc(const icar_interfaces::msg::Stm32FromPc::SharedPtr msg) {
    throttle = msg->throttle;
    steering = msg->steering;
    transmission = msg->transmission;
    accessory = msg->accessory;
  }

  //====================================

  bool stm32_init() {
    RCLCPP_INFO(this->get_logger(), "IP: %s", stm32_ip.c_str());
    RCLCPP_INFO(this->get_logger(), "Port: %d", stm32_port);

    if (stm32_udp.init_as_client(stm32_ip, stm32_port) == false) {
      RCLCPP_ERROR(this->get_logger(), "STM32 UDP init failed");
      return false;
    }

    return true;
  }

  bool stm32_routine() {
    // Communication Protocol (PC -> STM32)
    // ====================================
    // Offset   | Size  | Description
    // 0        | 4     | epoch_from_pc
    // 4        | 2     | throttle
    // 6        | 2     | steering
    // 8        | 1     | transmission
    // 10       | 1     | accessory
    memcpy(stm32_tx_buffer + 0, &epoch_from_pc, 4);
    memcpy(stm32_tx_buffer + 4, &throttle, 2);
    memcpy(stm32_tx_buffer + 6, &steering, 2);
    memcpy(stm32_tx_buffer + 8, &transmission, 1);
    memcpy(stm32_tx_buffer + 10, &accessory, 1);

    int len_data_from_pc = stm32_udp.send(stm32_tx_buffer, 12);
    int len_data_to_pc = stm32_udp.recv(stm32_rx_buffer, 48);

    // Communication Protocol (STM32 -> PC)
    // ====================================
    // Offset   | Size  | Description
    // 0        | 4     | epoch_to_pc
    // 4        | 32    | remote_control
    // 36       | 2     | encoder_kiri
    // 38       | 2     | encoder_kanan
    // 40       | 4     | gyroscope
    // 44       | 2     | throttle_position
    // 46       | 2     | steering_position
    memcpy(&epoch_to_pc, stm32_rx_buffer + 0, 4);
    memcpy(&remote_control, stm32_rx_buffer + 4, 32);
    memcpy(&encoder_kiri, stm32_rx_buffer + 36, 2);
    memcpy(&encoder_kanan, stm32_rx_buffer + 38, 2);
    memcpy(&gyroscope, stm32_rx_buffer + 40, 4);
    memcpy(&throttle_position, stm32_rx_buffer + 44, 2);
    memcpy(&steering_position, stm32_rx_buffer + 46, 2);

    // =================================

    if (len_data_from_pc < 0) { return true; }
    if (len_data_to_pc < 0) { return true; }

    // =================================

    epoch_from_pc++;

    // =================================

    static uint32_t last_epoch_to_pc = 0;

    if (last_epoch_to_pc == 0) {
      last_epoch_to_pc = epoch_to_pc;
      return true;
    }

    if (epoch_to_pc != last_epoch_to_pc + 1) {
      last_epoch_to_pc = epoch_to_pc;
      return true;
    }

    last_epoch_to_pc = epoch_to_pc;

    // =================================

    static icar_interfaces::msg::Stm32ToPc msg_stm32_to_pc;
    // ----
    msg_stm32_to_pc.remote_control = std::vector<uint16_t>(remote_control, remote_control + 16);
    msg_stm32_to_pc.encoder_kiri = encoder_kiri;
    msg_stm32_to_pc.encoder_kanan = encoder_kanan;
    msg_stm32_to_pc.gyroscope = gyroscope;
    msg_stm32_to_pc.throttle_position = throttle_position;
    msg_stm32_to_pc.steering_position = steering_position;
    // ----
    pub_stm32_to_pc->publish(msg_stm32_to_pc);

    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_io_stm32 = std::make_shared<IOSTM32>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_io_stm32);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}