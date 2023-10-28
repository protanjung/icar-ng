#include "pandu_ros2_kit/udp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class IOLSLIDARN301 : public rclcpp::Node {
 public:
  //-----Parameter
  int msop_port;
  int difop_port;
  std::string frame_id;
  float azimuth_start;
  float azimuth_stop;
  float distance_min;
  float distance_max;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_2000hz;
  //-----Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_xyz;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_xyzi;

  // Socket connection
  // =================
  UDP msop_udp;
  uint8_t msop_tx_buffer[2048];
  uint8_t msop_rx_buffer[2048];
  UDP difop_udp;
  uint8_t difop_tx_buffer[2048];
  uint8_t difop_rx_buffer[2048];

  // Lidar data
  // ==========
  float azimuth_cos_table[36000];
  float azimuth_sin_table[36000];

  typedef struct {
    uint8_t distance[2];
    uint8_t intensity;
  } msop_data_t;

  typedef struct {
    uint8_t flag[2];
    uint8_t azimuth[2];
    msop_data_t data[32];
  } msop_block_t;

  typedef struct {
    msop_block_t block[12];
    uint8_t timestamp[4];
    uint8_t factory[2];
  } msop_packet_t;

  pcl::PointCloud<pcl::PointXYZ> points_xyz;
  pcl::PointCloud<pcl::PointXYZI> points_xyzi;

  IOLSLIDARN301() : Node("io_lslidar_n301") {
    //-----Parameter
    this->declare_parameter("msop_port", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("difop_port", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
    this->declare_parameter("azimuth_start", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("azimuth_stop", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("distance_min", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("distance_max", rclcpp::PARAMETER_DOUBLE);
    this->get_parameter("msop_port", msop_port);
    this->get_parameter("difop_port", difop_port);
    this->get_parameter("frame_id", frame_id);
    this->get_parameter("azimuth_start", azimuth_start);
    this->get_parameter("azimuth_stop", azimuth_stop);
    this->get_parameter("distance_min", distance_min);
    this->get_parameter("distance_max", distance_max);
    //-----Timer
    tim_2000hz = this->create_wall_timer(0.5ms, std::bind(&IOLSLIDARN301::cllbck_tim_2000hz, this));
    //-----Publisher
    pub_points_xyz = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_xyz", 1);
    pub_points_xyzi = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_xyzi", 1);

    if (lidar_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Lidar initialization failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_2000hz() {
    if (lidar_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Lidar routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  bool lidar_init() {
    RCLCPP_INFO(this->get_logger(), "MSOP Port: %d", msop_port);
    RCLCPP_INFO(this->get_logger(), "DIFOP Port: %d", difop_port);
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Azimuth Start: %f", azimuth_start);
    RCLCPP_INFO(this->get_logger(), "Azimuth Stop: %f", azimuth_stop);
    RCLCPP_INFO(this->get_logger(), "Distance Min: %f", distance_min);
    RCLCPP_INFO(this->get_logger(), "Distance Max: %f", distance_max);

    if (msop_udp.init_as_server(msop_port) == false) {
      RCLCPP_ERROR(this->get_logger(), "MSOP UDP init failed");
      return false;
    }

    if (difop_udp.init_as_server(difop_port) == false) {
      RCLCPP_ERROR(this->get_logger(), "DIFOP UDP init failed");
      return false;
    }

    for (int i = 0; i < 36000; i++) {
      azimuth_cos_table[i] = cos(i * M_PI / 18000);
      azimuth_sin_table[i] = -sin(i * M_PI / 18000);
    }

    return true;
  }

  bool lidar_routine() {
    if (msop_udp.recv(msop_rx_buffer, 1206) == 1206) { parse_msop(); }
    if (difop_udp.recv(difop_rx_buffer, 1206) == 1206) { parse_difop(); }

    return true;
  }

  //====================================

  bool parse_msop() {
    msop_packet_t *packet = (msop_packet_t *)msop_rx_buffer;

    for (int i_block = 0; i_block < 12; i_block++) {
      static const uint8_t flag[2] = {0xFF, 0xEE};
      if (memcmp(packet->block[i_block].flag, flag, 2) != 0) { continue; }

      /* This code snippet is used to calculate the azimuth angle for each block of lidar data. */
      static float last_azimuth_raw = 0;
      static float azimuth_raw = 0;
      last_azimuth_raw = azimuth_raw;
      azimuth_raw = (float)(*(uint16_t *)packet->block[i_block].azimuth) / 100;

      /* This code calculates the difference in azimuth angles between consecutive blocks of lidar data. */
      float azimuth_difference = 0;
      if (i_block > 0) {
        uint16_t azimuth0 = *(uint16_t *)packet->block[i_block].azimuth;
        uint16_t azimuth1 = *(uint16_t *)packet->block[i_block - 1].azimuth;
        azimuth_difference = (float)((azimuth0 - azimuth1 + 36000) % 36000) / 100;
      } else {
        uint16_t azimuth0 = *(uint16_t *)packet->block[i_block].azimuth;
        uint16_t azimuth1 = *(uint16_t *)packet->block[i_block + 1].azimuth;
        azimuth_difference = (float)((azimuth1 - azimuth0 + 36000) % 36000) / 100;
      }

      for (int i_laser = 0; i_laser < 2; i_laser++) {
        for (int i_data = 0; i_data < 15; i_data++) {
          /* The line of code is calculating the corrected azimuth angle for each laser point in the lidar data. */
          float azimuth_correct = azimuth_raw + (azimuth_difference / 30) * (i_laser * 15 + i_data);
          azimuth_correct = azimuth_correct > 360 ? azimuth_correct - 360 : azimuth_correct;

          /* This code is extracting the raw distance and intensity values from the lidar data packet and then
          converting them to correct values. */
          uint16_t distance_raw = *(uint16_t *)packet->block[i_block].data[i_laser * 16 + i_data].distance;
          uint8_t intensity_raw = packet->block[i_block].data[i_laser * 16 + i_data].intensity;
          float distance_correct = (float)distance_raw * 0.002;
          float intensity_correct = (float)intensity_raw / 255;

          /* This code is checking if the azimuth angle and distance values of a lidar point fall within the
          specified range. If the azimuth angle is outside the range (either less than the start angle or
          greater than the stop angle), or if the distance is outside the range (either less than the minimum
          distance or greater than the maximum distance), the code will skip processing that lidar point and
          continue to the next one. */
          if (azimuth_start < azimuth_stop && (azimuth_correct < azimuth_start || azimuth_correct > azimuth_stop)) {
            continue;
          }
          if (azimuth_stop < azimuth_start && (azimuth_correct < azimuth_start && azimuth_correct > azimuth_stop)) {
            continue;
          }
          if (distance_correct < distance_min || distance_correct > distance_max) { continue; }

          /* The line of code `uint16_t azimuth_index = (uint16_t)(azimuth_correct * 100) % 36000;` is
          calculating the index of the azimuth angle in the azimuth cosine and sine lookup tables. */
          uint16_t azimuth_index = (uint16_t)(azimuth_correct * 100) % 36000;

          pcl::PointXYZ point_xyz;
          pcl::PointXYZI point_xyzi;
          point_xyz.x = point_xyzi.x = distance_correct * azimuth_cos_table[azimuth_index];
          point_xyz.y = point_xyzi.y = distance_correct * azimuth_sin_table[azimuth_index];
          point_xyz.z = point_xyzi.z = 0;
          point_xyzi.intensity = intensity_correct;
          points_xyz.push_back(point_xyz);
          points_xyzi.push_back(point_xyzi);
        }
      }

      if (azimuth_raw < last_azimuth_raw) {
        sensor_msgs::msg::PointCloud2 msg_points_xyz;
        sensor_msgs::msg::PointCloud2 msg_points_xyzi;
        pcl::toROSMsg(points_xyz, msg_points_xyz);
        pcl::toROSMsg(points_xyzi, msg_points_xyzi);
        msg_points_xyz.header.frame_id = msg_points_xyzi.header.frame_id = frame_id;
        msg_points_xyz.header.stamp = msg_points_xyzi.header.stamp = this->now();
        pub_points_xyz->publish(msg_points_xyz);
        pub_points_xyzi->publish(msg_points_xyzi);

        points_xyz.clear();
        points_xyzi.clear();
      }
    }

    return true;
  }

  bool parse_difop() {
    static const uint8_t header[] = {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55};
    static const uint8_t tail[] = {0x0F, 0xF0};
    if (memcmp(difop_rx_buffer, header, 8) != 0) { return false; }
    if (memcmp(difop_rx_buffer + 1204, tail, 2) != 0) { return false; }

    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_io_lslidar_n301 = std::make_shared<IOLSLIDARN301>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_io_lslidar_n301);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}