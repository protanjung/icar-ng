#include "GeographicLib/UTMUPS.hpp"
#include "icar_interfaces/msg/gps_to_pc.hpp"
#include "libserialport.h"
#include "lwgps/lwgps.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace GeographicLib;

class IOGPS : public rclcpp::Node {
 public:
  //-----Parameter
  std::string gps_port;
  int gps_baud;
  double gps_origin_lat;
  double gps_origin_lon;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_100hz;
  //-----Publisher
  rclcpp::Publisher<icar_interfaces::msg::GpsToPc>::SharedPtr pub_gps_to_pc;

  // Serial connection
  // =================
  struct sp_port *serial_port;
  uint16_t serial_tx_len;
  uint16_t serial_rx_len;
  uint8_t serial_tx_buffer[1024];
  uint8_t serial_rx_buffer[1024];

  // Global Positioning System
  // =========================
  lwgps_t gps;

  // GPS Origin
  // ==========
  int gps_origin_zone;
  bool gps_origin_northp;
  double gps_origin_x;
  double gps_origin_y;

  // GPS Pose
  // ========
  int gps_pose_zone;
  bool gps_pose_northp;
  double gps_pose_x;
  double gps_pose_y;

  IOGPS() : Node("io_gps") {
    //-----Parameter
    this->declare_parameter("gps.port", rclcpp::PARAMETER_STRING);
    this->declare_parameter("gps.baud", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("gps.origin_lat", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("gps.origin_lon", rclcpp::PARAMETER_DOUBLE);
    this->get_parameter("gps.port", gps_port);
    this->get_parameter("gps.baud", gps_baud);
    this->get_parameter("gps.origin_lat", gps_origin_lat);
    this->get_parameter("gps.origin_lon", gps_origin_lon);
    //-----Timer
    tim_100hz = this->create_wall_timer(10ms, std::bind(&IOGPS::cllbck_tim_100hz, this));
    //-----Publisher
    pub_gps_to_pc = this->create_publisher<icar_interfaces::msg::GpsToPc>("gps/to_pc", 10);

    if (gps_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "GPS initialization failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_100hz() {
    if (gps_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "GPS routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  bool gps_init() {
    RCLCPP_INFO(this->get_logger(), "Port: %s", gps_port.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud: %d", gps_baud);
    RCLCPP_INFO(this->get_logger(), "Origin Lat: %f", gps_origin_lat);
    RCLCPP_INFO(this->get_logger(), "Origin Lon: %f", gps_origin_lon);

    /* Initialize serial port */
    if (sp_get_port_by_name(realpath(gps_port.c_str(), NULL), &serial_port) != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Serial port not found");
      return false;
    }
    sp_close(serial_port);
    if (sp_open(serial_port, SP_MODE_READ_WRITE) != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Serial port open failed");
      return false;
    }
    if (sp_set_baudrate(serial_port, gps_baud) != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Serial port set baudrate failed");
      return false;
    }
    if (sp_set_bits(serial_port, 8) != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Serial port set bits failed");
      return false;
    }
    if (sp_set_parity(serial_port, SP_PARITY_NONE) != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Serial port set parity failed");
      return false;
    }
    if (sp_set_stopbits(serial_port, 1) != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Serial port set stopbits failed");
      return false;
    }

    /* Initialize GPS structure */
    lwgps_init(&gps);

    /* Convert origin coordinates to UTM */
    UTMUPS::Forward(gps_origin_lat, gps_origin_lon, gps_origin_zone, gps_origin_northp, gps_origin_x, gps_origin_y);

    return true;
  }

  bool gps_routine() {
    /* Convert current coordinate to UTM */
    UTMUPS::Forward(gps.latitude, gps.longitude, gps_pose_zone, gps_pose_northp, gps_pose_x, gps_pose_y);

    /* Reading data from the serial port and processing it using the lwgps library. */
    sp_return serial_return;
    uint8_t serial_data = 0;
    uint8_t gps_return = 0;
    while ((serial_return = sp_nonblocking_read(serial_port, &serial_data, 1)) > 0) {
      gps_return += lwgps_process(&gps, &serial_data, 1);
    }
    if (serial_return < 0) {
      // RCLCPP_ERROR(this->get_logger(), "Serial port read failed");
      return false;
    }
    if (gps_return < 1) {
      // RCLCPP_ERROR(this->get_logger(), "GPS process failed");
      return true;
    }

    static icar_interfaces::msg::GpsToPc msg_gps_to_pc;
    // ----
    msg_gps_to_pc.navsatfix.header.stamp = this->now();
    msg_gps_to_pc.navsatfix.header.frame_id = "gps_link";
    // ----
    msg_gps_to_pc.navsatfix.status.status = gps.fix == 0 || gps.fix_mode == 1
                                                ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX
                                                : sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    // ----
    msg_gps_to_pc.navsatfix.status.service =
        sensor_msgs::msg::NavSatStatus::SERVICE_GPS | sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
        sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS | sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
    // ----
    msg_gps_to_pc.navsatfix.latitude = gps.latitude;
    msg_gps_to_pc.navsatfix.longitude = gps.longitude;
    msg_gps_to_pc.navsatfix.altitude = gps.altitude;
    // ----
    msg_gps_to_pc.navsatfix.position_covariance[0] = gps.dop_h * gps.dop_h;
    msg_gps_to_pc.navsatfix.position_covariance[4] = gps.dop_h * gps.dop_h;
    msg_gps_to_pc.navsatfix.position_covariance[8] = gps.dop_v * gps.dop_v;
    msg_gps_to_pc.navsatfix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    // ----
    msg_gps_to_pc.pose2d.x = gps_pose_x - gps_origin_x;
    msg_gps_to_pc.pose2d.y = gps_pose_y - gps_origin_y;
    msg_gps_to_pc.pose2d.theta = gps.speed > 4.82 * 0.539957 ? (90 - gps.course) * M_PI / 180 : 0;
    // ----
    pub_gps_to_pc->publish(msg_gps_to_pc);

    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_io_gps = std::make_shared<IOGPS>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_io_gps);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}