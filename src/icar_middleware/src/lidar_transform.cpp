#include "boost/thread/mutex.hpp"
#include "icar_interfaces/msg/lidar_rings.hpp"
#include "pandu_ros2_kit/help_marker.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

class LidarTransform : public rclcpp::Node {
 public:
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_10hz;
  //-----Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_front_points_xyzir;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_rearright_points_xyzi;
  //-----Publisher
  rclcpp::Publisher<icar_interfaces::msg::LidarRings>::SharedPtr pub_lidar_base_rings;
  //-----Transform listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  //-----Mutex
  boost::mutex mutex_lidar_front;
  boost::mutex mutex_lidar_rearright;
  //-----Help
  HelpMarker _marker;

  // Transform
  // =========
  bool tf_is_initialized = false;
  geometry_msgs::msg::TransformStamped tf_base_lidar_front;
  geometry_msgs::msg::TransformStamped tf_base_lidar_rearright;

  // Point cloud
  // ===========
  pcl::PointCloud<pcl::PointXYZ> lidar_front_points_all, lidar_front_points_all_on_base;
  pcl::PointCloud<pcl::PointXYZ> lidar_front_points_ring[16], lidar_front_points_ring_on_base[16];
  pcl::PointCloud<pcl::PointXYZ> lidar_rearright_points_all, lidar_rearright_points_all_on_base;

  // Lidar ring
  // ==========
  const float lidar_base_rings_angle_start = -M_PI_2;
  const float lidar_base_rings_angle_stop = M_PI_2;
  const float lidar_base_rings_angle_step = (lidar_base_rings_angle_stop - lidar_base_rings_angle_start) / 90;
  icar_interfaces::msg::LidarRings lidar_base_rings_default;

  LidarTransform() : Node("lidar_transform") {
    //-----Timer
    tim_10hz = this->create_wall_timer(100ms, std::bind(&LidarTransform::cllbck_tim_10hz, this));
    //-----Subscriber
    sub_lidar_front_points_xyzir = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar_front/points_xyzir",
        10,
        std::bind(&LidarTransform::cllbck_sub_lidar_front_points_xyzir, this, std::placeholders::_1));
    sub_lidar_rearright_points_xyzi = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar_rearright/points_xyzi",
        10,
        std::bind(&LidarTransform::cllbck_sub_lidar_rearright_xyzi, this, std::placeholders::_1));
    //-----Publisher
    pub_lidar_base_rings = this->create_publisher<icar_interfaces::msg::LidarRings>("lidar_base/rings", 10);
    //-----Tranform listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    if (lidar_transform_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Lidar transform init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_10hz() {
    if (lidar_transform_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Lidar transform routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_lidar_front_points_xyzir(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!tf_is_initialized) { return; }

    /* Converting ROS message to PCL point cloud */
    pcl::PointCloud<PointXYZIR> input;
    pcl::fromROSMsg(*msg, input);

    /* Clearing the previous data */
    lidar_front_points_all.clear();
    for (int i = 0; i < 16; i++) { lidar_front_points_ring[i].clear(); }

    /* Append all the points to the point cloud */
    for (auto point : input.points) {
      pcl::PointXYZ p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      lidar_front_points_all.push_back(p);
      lidar_front_points_ring[point.ring].push_back(p);
    }

    /* Transform the point cloud to the base_link frame */
    mutex_lidar_front.lock();
    pcl_ros::transformPointCloud(lidar_front_points_all, lidar_front_points_all_on_base, tf_base_lidar_front);
    for (int i = 0; i < 16; i++) {
      pcl_ros::transformPointCloud(lidar_front_points_ring[i], lidar_front_points_ring_on_base[i], tf_base_lidar_front);
    }
    mutex_lidar_front.unlock();
  }

  void cllbck_sub_lidar_rearright_xyzi(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!tf_is_initialized) { return; }

    /* Converting ROS message to PCL point cloud */
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::fromROSMsg(*msg, input);

    /* Clearing the previous data */
    lidar_rearright_points_all.clear();

    /* Append all the points to the point cloud */
    for (auto point : input.points) {
      pcl::PointXYZ p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      lidar_rearright_points_all.push_back(p);
    }

    /* Transform the point cloud to the base_link frame */
    mutex_lidar_rearright.lock();
    pcl_ros::transformPointCloud(
        lidar_rearright_points_all, lidar_rearright_points_all_on_base, tf_base_lidar_rearright);
    mutex_lidar_rearright.unlock();
  }

  //====================================

  bool lidar_transform_init() {
    while (!tf_is_initialized) {
      try {
        tf_base_lidar_front = tf_buffer->lookupTransform("base_link", "lidar_front_link", tf2::TimePointZero);
        tf_base_lidar_rearright = tf_buffer->lookupTransform("base_link", "lidar_rearright_link", tf2::TimePointZero);
        tf_is_initialized = true;
      } catch (...) { std::this_thread::sleep_for(1s); }
    }

    icar_interfaces::msg::LidarRings lidar_rings;
    for (int i = 0; i < 16; i++) {
      icar_interfaces::msg::LidarRing lidar_ring;
      for (int i = 0; i < 90; i++) {
        geometry_msgs::msg::Point p;
        p.x = tf_base_lidar_front.transform.translation.x;
        p.y = tf_base_lidar_front.transform.translation.y;
        p.z = tf_base_lidar_front.transform.translation.z;
        lidar_ring.point.push_back(p);
      }
      lidar_rings.ring.push_back(lidar_ring);
    }
    lidar_base_rings_default = lidar_rings;

    return true;
  }

  bool lidar_transform_routine() {
    if (!_marker.is_initialized()) { _marker.init(this->shared_from_this()); }

    lidar_base_rings_routine();

    return true;
  }

  //====================================

  void lidar_base_rings_routine() {
    mutex_lidar_front.lock();

    icar_interfaces::msg::LidarRings msg_lidar_base_rings = lidar_base_rings_default;

    for (int i_ring = 0; i_ring < 16; i_ring++) {
      for (auto point : lidar_front_points_ring_on_base[i_ring]) {
        /* Calculate the distance of the point to the origin */
        float dx = point.x - tf_base_lidar_front.transform.translation.x;
        float dy = point.y - tf_base_lidar_front.transform.translation.y;
        float r = sqrtf(dx * dx + dy * dy);

        /* Calculate the angle of the point */
        float angle = atan2f(dy, dx);
        /* Calculate the index of the angle in the lidar rings */
        int angle_index = (angle - lidar_base_rings_angle_start) / lidar_base_rings_angle_step;
        /* If the angle is out of range, skip the point */
        if (angle_index < 0 || angle_index >= 90) { continue; }

        /* Calculate the existing point's distance to the origin */
        float dx_old =
            msg_lidar_base_rings.ring[i_ring].point[angle_index].x - tf_base_lidar_front.transform.translation.x;
        float dy_old =
            msg_lidar_base_rings.ring[i_ring].point[angle_index].y - tf_base_lidar_front.transform.translation.y;
        float r_old = sqrtf(dx_old * dx_old + dy_old * dy_old);

        /* If the new point is closer to the origin, replace the existing point */
        if (r < r_old || r_old == 0.0) {
          msg_lidar_base_rings.ring[i_ring].point[angle_index].x = point.x;
          msg_lidar_base_rings.ring[i_ring].point[angle_index].y = point.y;
          msg_lidar_base_rings.ring[i_ring].point[angle_index].z = point.z;
        }
      }
    }

    pub_lidar_base_rings->publish(msg_lidar_base_rings);

    mutex_lidar_front.unlock();
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node_lidar_transform = std::make_shared<LidarTransform>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_lidar_transform);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}