#include "pandu_ros2_kit/udp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// ! This code defines a custom point type called `PointXYZIR` for a Point Cloud Library (PCL) point cloud.
struct EIGEN_ALIGN16 PointXYZIR {
  float x;
  float y;
  float z;
  float intensity;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, ring, ring))
// ! This code defines a custom point type called `PointXYZIR` for a Point Cloud Library (PCL) point cloud.

using namespace std::chrono_literals;

class IOLSLIDARC16 : public rclcpp::Node {
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_xyzir;

  // Socket connection
  // =================
  UDP udp_msop;
  UDP udp_difop;
  uint8_t msop_tx_buffer[2048];
  uint8_t msop_rx_buffer[2048];
  uint8_t difop_tx_buffer[2048];
  uint8_t difop_rx_buffer[2048];

  // Lidar data
  // ==========
  float elevation_table[16] = {
      -0.261799387799149,
      0.0174532925199433,
      -0.226892802759263,
      0.0523598775598299,
      -0.191986217719376,
      0.0872664625997165,
      -0.15707963267949,
      0.122173047639603,
      -0.122173047639603,
      0.15707963267949,
      -0.0872664625997165,
      0.191986217719376,
      -0.0523598775598299,
      0.226892802759263,
      -0.0174532925199433,
      0.261799387799149};
  uint8_t ring_table[16] = {0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15};
  float azimuth_cos_table[36000];
  float azimuth_sin_table[36000];
  float elevation_cos_table[16];
  float elevation_sin_table[16];

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
  pcl::PointCloud<PointXYZIR> points_xyzir;

  IOLSLIDARC16() : Node("io_lslidar_c16") {
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
    tim_2000hz = this->create_wall_timer(0.5ms, std::bind(&IOLSLIDARC16::cllbck_tim_2000hz, this));
    //-----Publisher
    pub_points_xyz = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_xyz", 1);
    pub_points_xyzir = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_xyzir", 1);

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

    if (udp_msop.init_as_server(msop_port) == false) {
      RCLCPP_ERROR(this->get_logger(), "MSOP init failed");
      return false;
    }

    if (udp_difop.init_as_server(difop_port) == false) {
      RCLCPP_ERROR(this->get_logger(), "DIFOP init failed");
      return false;
    }

    for (int i = 0; i < 36000; i++) {
      azimuth_cos_table[i] = cos(i * M_PI / 18000);
      azimuth_sin_table[i] = sin(i * M_PI / 18000);
    }

    for (int i = 0; i < 16; i++) {
      elevation_cos_table[i] = cos(elevation_table[i]);
      elevation_sin_table[i] = sin(elevation_table[i]);
    }

    return true;
  }

  bool lidar_routine() {
    if (udp_msop.recv(msop_rx_buffer, 1206) == 1206) { parse_msop(); }
    if (udp_difop.recv(difop_rx_buffer, 1206) == 1206) { parse_difop(); }

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
        for (int i_data = 0; i_data < 16; i_data++) {
          /* The line of code is calculating the corrected azimuth angle for each laser point in the lidar data. */
          float azimuth_correct = azimuth_raw + (azimuth_difference / 32) * (i_laser * 16 + i_data);

          /* This code is extracting the raw distance and intensity values from the lidar data packet and then
          converting them to correct values. */
          uint16_t distance_raw = *(uint16_t *)packet->block[i_block].data[i_laser * 16 + i_data].distance;
          uint8_t intensity_raw = packet->block[i_block].data[i_laser * 16 + i_data].intensity;
          float distance_correct = (float)distance_raw * 0.0025;
          float intensity_correct = (float)intensity_raw / 255;

          /* The line `uint8_t ring_correct = ring_table[i_data];` is assigning the value from the `ring_table`
          array to the variable `ring_correct`. The `ring_table` array is used to map the index `i_data` to
          the correct ring value for a lidar point. The `ring_correct` variable will hold the correct ring
          value for each lidar point in the loop. */
          uint8_t ring_correct = ring_table[i_data];

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
          PointXYZIR point_xyzir;
          point_xyz.x = point_xyzir.x =
              distance_correct * azimuth_cos_table[azimuth_index] * elevation_cos_table[i_data];
          point_xyz.y = point_xyzir.y =
              distance_correct * azimuth_sin_table[azimuth_index] * elevation_cos_table[i_data];
          point_xyz.z = point_xyzir.z = distance_correct * elevation_sin_table[i_data];
          point_xyzir.intensity = intensity_correct;
          point_xyzir.ring = ring_correct;
          points_xyz.push_back(point_xyz);
          points_xyzir.push_back(point_xyzir);
        }
      }

      if (azimuth_raw < last_azimuth_raw) {
        sensor_msgs::msg::PointCloud2 msg_points_xyz;
        sensor_msgs::msg::PointCloud2 msg_points_xyzir;
        pcl::toROSMsg(points_xyz, msg_points_xyz);
        pcl::toROSMsg(points_xyzir, msg_points_xyzir);
        msg_points_xyz.header.frame_id = msg_points_xyzir.header.frame_id = frame_id;
        msg_points_xyz.header.stamp = msg_points_xyzir.header.stamp = this->now();
        pub_points_xyz->publish(msg_points_xyz);
        pub_points_xyzir->publish(msg_points_xyzir);

        points_xyz.clear();
        points_xyzir.clear();
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

  auto node_io_lslidar_c16 = std::make_shared<IOLSLIDARC16>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_io_lslidar_c16);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}