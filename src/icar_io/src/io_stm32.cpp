#include "arpa/inet.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

class IOSTM32 : public rclcpp::Node {
 public:
  //-----Parameter
  std::string stm32_ip;
  int stm32_port;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_50hz;
  //-----Subscriber
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_stm32frompc_throttle_steering;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_stm32frompc_transmission;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_stm32frompc_accessory;
  //-----Publisher
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_stm32topc_remote;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_stm32topc_encoder;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_stm32topc_gyroscope;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_stm32topc_throttle_steering_position;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_stm32topc_mode;

  // Socket connection
  // =================
  int socket_fd = 0;
  struct sockaddr_in socket_server_address;
  struct sockaddr_in socket_client_address;
  uint16_t socket_tx_len;
  uint16_t socket_rx_len;
  uint8_t socket_tx_buffer[1024];
  uint8_t socket_rx_buffer[1024];

  // Data to PC
  // ==========
  uint32_t epoch_to_pc;
  uint16_t remote[16];
  uint16_t encoder_kiri;
  uint16_t encoder_kanan;
  float gyroscope;
  int16_t throttle_position;
  int16_t steering_position;
  uint8_t mode;

  // Data from PC
  // ============
  uint32_t epoch_from_pc;
  int16_t throttle;
  int16_t steering;
  int8_t transmission;
  uint8_t accessory;

  IOSTM32() : Node("io_stm32") {
    //-----Parameter
    this->declare_parameter("stm32_ip", "293.268.50.2");
    this->declare_parameter("stm32_port", 9798);
    this->get_parameter("stm32_ip", stm32_ip);
    this->get_parameter("stm32_port", stm32_port);
    //-----Timer
    tim_50hz = this->create_wall_timer(20ms, std::bind(&IOSTM32::cllbck_tim_50hz, this));
    //-----Subscriber
    sub_stm32frompc_throttle_steering = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "stm32frompc/throttle_steering",
        10,
        std::bind(&IOSTM32::cllbck_sub_stm32frompc_throttle_steering, this, std::placeholders::_1));
    sub_stm32frompc_transmission = this->create_subscription<std_msgs::msg::Int8>(
        "stm32frompc/transmission",
        10,
        std::bind(&IOSTM32::cllbck_sub_stm32frompc_transmission, this, std::placeholders::_1));
    sub_stm32frompc_accessory = this->create_subscription<std_msgs::msg::UInt8>(
        "stm32frompc/accessory",
        10,
        std::bind(&IOSTM32::cllbck_sub_stm32frompc_accessory, this, std::placeholders::_1));
    //-----Publisher
    pub_stm32topc_remote = this->create_publisher<std_msgs::msg::UInt16MultiArray>("stm32topc/remote", 10);
    pub_stm32topc_encoder = this->create_publisher<std_msgs::msg::UInt16MultiArray>("stm32topc/encoder", 10);
    pub_stm32topc_gyroscope = this->create_publisher<std_msgs::msg::Float32>("stm32topc/gyroscope", 10);
    pub_stm32topc_throttle_steering_position =
        this->create_publisher<std_msgs::msg::Int16MultiArray>("stm32topc/throttle_steering_position", 10);
    pub_stm32topc_mode = this->create_publisher<std_msgs::msg::UInt8>("stm32topc/mode", 10);

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

  void cllbck_sub_stm32frompc_throttle_steering(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (msg->data.size() != 2) { return; }
    throttle = msg->data[0];
    steering = msg->data[1];
  }

  void cllbck_sub_stm32frompc_transmission(const std_msgs::msg::Int8::SharedPtr msg) { transmission = msg->data; }

  void cllbck_sub_stm32frompc_accessory(const std_msgs::msg::UInt8::SharedPtr msg) { accessory = msg->data; }

  //====================================

  bool stm32_init() {
    RCLCPP_INFO(this->get_logger(), "IP: %s", stm32_ip.c_str());
    RCLCPP_INFO(this->get_logger(), "Port: %d", stm32_port);

    /* The code is creating a socket for communication with the STM32 microcontroller. */
    if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
      return false;
    }

    /* The code is setting up the server address for the socket communication with the STM32
    microcontroller. */
    socket_server_address.sin_family = AF_INET;
    socket_server_address.sin_addr.s_addr = inet_addr(stm32_ip.c_str());
    socket_server_address.sin_port = htons(stm32_port);
    socket_tx_len = sizeof(socket_server_address);

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
    memcpy(socket_tx_buffer + 0, &epoch_from_pc, 4);
    memcpy(socket_tx_buffer + 4, &throttle, 2);
    memcpy(socket_tx_buffer + 6, &steering, 2);
    memcpy(socket_tx_buffer + 8, &transmission, 1);
    memcpy(socket_tx_buffer + 10, &accessory, 1);

    /* The code is sending data to the STM32 device. */
    int len_data_from_pc = sendto(
        socket_fd,
        socket_tx_buffer,
        12,
        MSG_DONTWAIT,
        (const struct sockaddr *)&socket_server_address,
        (socklen_t)socket_tx_len);

    /* The code is receiving data from the STM32 device. */
    int len_data_to_pc = recvfrom(
        socket_fd,
        socket_rx_buffer,
        1024,
        MSG_DONTWAIT,
        (struct sockaddr *)&socket_client_address,
        (socklen_t *)&socket_rx_len);

    // Communication Protocol (STM32 -> PC)
    // ====================================
    // Offset   | Size  | Description
    // 0        | 4     | epoch_to_pc
    // 4        | 32    | remote
    // 36       | 2     | encoder_kiri
    // 38       | 2     | encoder_kanan
    // 40       | 4     | gyroscope
    // 44       | 2     | throttle_position
    // 46       | 2     | steering_position
    // 48       | 1     | mode
    memcpy(&epoch_to_pc, socket_rx_buffer + 0, 4);
    memcpy(&remote, socket_rx_buffer + 4, 32);
    memcpy(&encoder_kiri, socket_rx_buffer + 36, 2);
    memcpy(&encoder_kanan, socket_rx_buffer + 38, 2);
    memcpy(&gyroscope, socket_rx_buffer + 40, 4);
    memcpy(&throttle_position, socket_rx_buffer + 44, 2);
    memcpy(&steering_position, socket_rx_buffer + 46, 2);
    memcpy(&mode, socket_rx_buffer + 48, 1);

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

    std_msgs::msg::UInt16MultiArray msg_stm32topc_remote;
    for (int i = 0; i < 16; i++) { msg_stm32topc_remote.data.push_back(remote[i]); }
    pub_stm32topc_remote->publish(msg_stm32topc_remote);

    std_msgs::msg::UInt16MultiArray msg_stm32topc_encoder;
    msg_stm32topc_encoder.data.push_back(encoder_kiri);
    msg_stm32topc_encoder.data.push_back(encoder_kanan);
    pub_stm32topc_encoder->publish(msg_stm32topc_encoder);

    std_msgs::msg::Float32 msg_stm32topc_gyroscope;
    msg_stm32topc_gyroscope.data = gyroscope;
    pub_stm32topc_gyroscope->publish(msg_stm32topc_gyroscope);

    std_msgs::msg::Int16MultiArray msg_stm32topc_throttle_steering_position;
    msg_stm32topc_throttle_steering_position.data.push_back(throttle_position);
    msg_stm32topc_throttle_steering_position.data.push_back(steering_position);
    pub_stm32topc_throttle_steering_position->publish(msg_stm32topc_throttle_steering_position);

    std_msgs::msg::UInt8 msg_stm32topc_mode;
    msg_stm32topc_mode.data = mode;
    pub_stm32topc_mode->publish(msg_stm32topc_mode);

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