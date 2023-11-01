#include "boost/thread/mutex.hpp"
#include "icar_interfaces/msg/lidar_rings.hpp"
#include "icar_interfaces/msg/pose.hpp"
#include "pandu_ros2_kit/help_marker.hpp"
#include "pcl/filters/passthrough.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class RoadSegmentation : public rclcpp::Node {
 public:
  //-----Timera
  rclcpp::TimerBase::SharedPtr tim_10hz;
  //-----Susbcriber
  rclcpp::Subscription<icar_interfaces::msg::LidarRings>::SharedPtr sub_lidar_base_rings;
  //-----Transform listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  //-----Help
  HelpMarker _marker;

  // Transform
  // =========
  bool tf_is_initialized = false;
  geometry_msgs::msg::TransformStamped tf_base_lidar_front;

  // Point cloud
  // ===========
  pcl::PointCloud<pcl::PointXYZ> cloud_left, cloud_pool_left[20];
  pcl::PointCloud<pcl::PointXYZ> cloud_right, cloud_pool_right[20];

  RoadSegmentation() : Node("road_segmentation") {
    //-----Timer
    tim_10hz = this->create_wall_timer(100ms, std::bind(&RoadSegmentation::cllbck_tim_10hz, this));
    //-----Subscriber
    sub_lidar_base_rings = this->create_subscription<icar_interfaces::msg::LidarRings>(
        "lidar_base/rings", 10, std::bind(&RoadSegmentation::cllbck_sub_lidar_base_rings, this, std::placeholders::_1));
    //-----Tranform listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    if (road_segmentation_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Road segmentation init failed");
      rclcpp::shutdown();
    }
  }

  // ===================================

  void cllbck_tim_10hz() {
    if (road_segmentation_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Road segmentation routine failed");
      rclcpp::shutdown();
    }
  }

  // ===================================

  void cllbck_sub_lidar_base_rings(const icar_interfaces::msg::LidarRings::SharedPtr msg) {
    if (!tf_is_initialized) { return; }

    static const float origin_x = tf_base_lidar_front.transform.translation.x;
    static const float origin_y = tf_base_lidar_front.transform.translation.y;

    // * Delta height
    // * ============
    pcl::PointCloud<pcl::PointXYZ> cloud_delta_height_left, cloud_delta_height_right;

    for (int i = 0; i < 6; i++) {
      pcl::PointCloud<pcl::PointXYZ> step0;
      pcl::PointCloud<pcl::PointXYZ> step1;
      pcl::PointXYZ step2_left, step2_right;
      pcl::PointXYZ step3_left, step3_right;

      ring_to_cloud(msg->ring[i], step0);
      rotate_cloud(step0, step1, 0, origin_x, origin_y);
      find_delta_height(step1, step2_left, step2_right);
      rotate_point(step2_left, step3_left, 0, origin_x, origin_y);
      rotate_point(step2_right, step3_right, 0, origin_x, origin_y);

      if (step2_left.y != 0) { cloud_delta_height_left.push_back(step3_left); }
      if (step2_right.y != 0) { cloud_delta_height_right.push_back(step3_right); }
    }

    // * Delta length
    // * ============
    pcl::PointCloud<pcl::PointXYZ> cloud_delta_length_left, cloud_delta_length_right;

    for (int i = 6; i < 10; i++) {
      pcl::PointCloud<pcl::PointXYZ> step0;
      pcl::PointCloud<pcl::PointXYZ> step1;
      pcl::PointXYZ step2_left, step2_right;
      pcl::PointXYZ step3_left, step3_right;

      ring_to_cloud(msg->ring[i], step0);
      rotate_cloud(step0, step1, 0, origin_x, origin_y);
      find_delta_length(step1, step2_left, step2_right);
      rotate_point(step2_left, step3_left, 0, origin_x, origin_y);
      rotate_point(step2_right, step3_right, 0, origin_x, origin_y);

      if (step2_left.y != 0) { cloud_delta_length_left.push_back(step3_left); }
      if (step2_right.y != 0) { cloud_delta_length_right.push_back(step3_right); }
    }

    // * Tangent angle
    // * =============
    pcl::PointCloud<pcl::PointXYZ> cloud_tangent_angle_left, cloud_tangent_angle_right;

    for (int i = 8; i < 10; i++) {
      pcl::PointXYZ step0_left, step0_right;

      find_tangent_angle(msg->ring[i], step0_left, step0_right, 0, origin_x, origin_y);

      cloud_tangent_angle_left.push_back(step0_left);
      cloud_tangent_angle_right.push_back(step0_right);
    }

    // // * Pooling
    // // * =======
    // cloud_left = cloud_delta_height_left + cloud_delta_length_left + cloud_tangent_angle_left;
    // cloud_right = cloud_delta_height_right + cloud_delta_length_right + cloud_tangent_angle_right;

    // * Visualization
    // * =============
    draw_cloud("road_candidate", 1, cloud_left, {0.5, 0.6, 0.5, 1.0});
    draw_cloud("road_candidate", 2, cloud_right, {1.0, 0.6, 0.5, 1.0});
  }

  // ===================================

  bool road_segmentation_init() {
    while (!tf_is_initialized) {
      try {
        tf_base_lidar_front = tf_buffer->lookupTransform("base_link", "lidar_front_link", tf2::TimePointZero);
        tf_is_initialized = true;
      } catch (...) { std::this_thread::sleep_for(1s); }
    }

    return true;
  }

  bool road_segmentation_routine() {
    if (!_marker.is_initialized()) { _marker.init(this->shared_from_this()); }

    return true;
  }

  // ===================================

  void find_delta_height(const pcl::PointCloud<pcl::PointXYZ> &in, pcl::PointXYZ &out_left, pcl::PointXYZ &out_right) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in.makeShared());
    pass.setFilterFieldName("y");

    // * Left side
    // * =========
    for (float y = 0.5; y < 5.0; y += 0.2) {  // ! FIX NOISE
      pcl::PointCloud<pcl::PointXYZ> temp;
      pass.setFilterLimits(y, y + 0.2);
      pass.filter(temp);

      float min_x = FLT_MAX, min_z = FLT_MAX;
      float max_x = -FLT_MAX, max_z = -FLT_MAX;
      for (const auto &point : temp.points) {
        if (point.x > max_x) { max_x = point.x; }
        if (point.x < min_x) { min_x = point.x; }
        if (point.z > max_z) { max_z = point.z; }
        if (point.z < min_z) { min_z = point.z; }
      }

      if (max_z - min_z > 0.08) {
        out_left.x = (max_x + min_x) / 2;
        out_left.y = y + 0.1;
        out_left.z = (max_z + min_z) / 2;
        break;
      }
    }

    // * Right side
    // * ==========
    for (float y = -0.5; y > -5.0; y -= 0.2) {  // ! FIX NOISE
      pcl::PointCloud<pcl::PointXYZ> temp;
      pass.setFilterLimits(y - 0.2, y);
      pass.filter(temp);

      float min_x = FLT_MAX, min_z = FLT_MAX;
      float max_x = -FLT_MAX, max_z = -FLT_MAX;
      for (const auto &point : temp.points) {
        if (point.x > max_x) { max_x = point.x; }
        if (point.x < min_x) { min_x = point.x; }
        if (point.z > max_z) { max_z = point.z; }
        if (point.z < min_z) { min_z = point.z; }
      }

      if (max_z - min_z > 0.08) {
        out_right.x = (max_x + min_x) / 2;
        out_right.y = y - 0.1;
        out_right.z = (max_z + min_z) / 2;
        break;
      }
    }
  }

  void find_delta_length(const pcl::PointCloud<pcl::PointXYZ> &in, pcl::PointXYZ &out_left, pcl::PointXYZ &out_right) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in.makeShared());
    pass.setFilterFieldName("y");

    // * Left side
    // * =========
    for (float y = 0.5; y < 5.0; y += 0.2) {  // ! FIX NOISE
      pcl::PointCloud<pcl::PointXYZ> temp;
      pass.setFilterLimits(y, y + 0.2);
      pass.filter(temp);

      float min_x = FLT_MAX, min_z = FLT_MAX;
      float max_x = -FLT_MAX, max_z = -FLT_MAX;
      for (const auto &point : temp.points) {
        if (point.x > max_x) { max_x = point.x; }
        if (point.x < min_x) { min_x = point.x; }
        if (point.z > max_z) { max_z = point.z; }
        if (point.z < min_z) { min_z = point.z; }
      }

      if (max_x - min_x > 0.12) {
        out_left.x = (max_x + min_x) / 2;
        out_left.y = y + 0.1;
        out_left.z = (max_z + min_z) / 2;
        break;
      }
    }

    // * Right side
    // * ==========
    for (float y = -0.5; y > -5.0; y -= 0.2) {  // ! FIX NOISE
      pcl::PointCloud<pcl::PointXYZ> temp;
      pass.setFilterLimits(y - 0.2, y);
      pass.filter(temp);

      float min_x = FLT_MAX, min_z = FLT_MAX;
      float max_x = -FLT_MAX, max_z = -FLT_MAX;
      for (const auto &point : temp.points) {
        if (point.x > max_x) { max_x = point.x; }
        if (point.x < min_x) { min_x = point.x; }
        if (point.z > max_z) { max_z = point.z; }
        if (point.z < min_z) { min_z = point.z; }
      }

      if (max_x - min_x > 0.12) {
        out_right.x = (max_x + min_x) / 2;
        out_right.y = y - 0.1;
        out_right.z = (max_z + min_z) / 2;
        break;
      }
    }
  }

  void find_tangent_angle(
      const icar_interfaces::msg::LidarRing &in,
      pcl::PointXYZ &out_left,
      pcl::PointXYZ &out_right,
      const double &angle,
      const double &origin_x,
      const double &origin_y) {
    static const float angle_start = -M_PI_2;
    static const float angle_stop = M_PI_2;
    static const float angle_step = (angle_start - angle_stop) / 90;

    int index_offset = angle / angle_step;
    if (index_offset < -43) { index_offset = -43; }
    if (index_offset > 43) { index_offset = 43; }

    // * Left side
    // * =========
    for (int i = index_offset + 50; i < 88; i++) {  // ! FIX NOISE
      float tangent0_x = in.point[i - 2].x;
      float tangent0_y = in.point[i - 2].y;
      float tangent1_x = in.point[i + 2].x;
      float tangent1_y = in.point[i + 2].y;
      float radial_x = in.point[i].x;
      float radial_y = in.point[i].y;

      float angle_tanget = atan2f(tangent1_y - tangent0_y, tangent1_x - tangent0_x);
      float angle_radial = atan2f(radial_y - origin_y, radial_x - origin_x);

      if (sinf(angle_tanget - angle_radial) < 0.80) {
        out_left.x = radial_x;
        out_left.y = radial_y;
        out_left.z = in.point[i].z;
        break;
      }
    }

    // * Right side
    // * ==========
    for (int i = index_offset + 40; i > 2; i--) {  // ! FIX NOISE
      float tangent0_x = in.point[i - 2].x;
      float tangent0_y = in.point[i - 2].y;
      float tangent1_x = in.point[i + 2].x;
      float tangent1_y = in.point[i + 2].y;
      float radial_x = in.point[i].x;
      float radial_y = in.point[i].y;

      float angle_tanget = atan2f(tangent1_y - tangent0_y, tangent1_x - tangent0_x);
      float angle_radial = atan2f(radial_y - origin_y, radial_x - origin_x);

      if (sinf(angle_tanget - angle_radial) < 0.80) {
        out_right.x = radial_x;
        out_right.y = radial_y;
        out_right.z = in.point[i].z;
        break;
      }
    }
  }

  // ===================================

  void draw_cloud(std::string ns, int id, pcl::PointCloud<pcl::PointXYZ> in, std::vector<float> color) {
    std::vector<geometry_msgs::msg::Point> points;
    for (const auto &point : in.points) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      points.push_back(p);
    }

    if (points.empty()) {
      _marker.points("base_link", ns, -id, points, color, 0.1, 0.1);
    } else {
      _marker.points("base_link", ns, id, points, color, 0.1, 0.1);
    }
  }

  void ring_to_cloud(const icar_interfaces::msg::LidarRing &in, pcl::PointCloud<pcl::PointXYZ> &out) {
    out.clear();
    for (const auto &point : in.point) {
      pcl::PointXYZ p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      out.push_back(p);
    }
  }

  void rotate_cloud(
      const pcl::PointCloud<pcl::PointXYZ> &in,
      pcl::PointCloud<pcl::PointXYZ> &out,
      const double &angle,
      const double &origin_x,
      const double &origin_y) {
    double s = sin(angle);
    double c = cos(angle);

    out.clear();
    for (const auto &point : in.points) {
      pcl::PointXYZ p;
      p.x = c * (point.x - origin_x) - s * (point.y - origin_y) + origin_x;
      p.y = s * (point.x - origin_x) + c * (point.y - origin_y) + origin_y;
      p.z = point.z;
      out.push_back(p);
    }
  }

  void rotate_point(
      const pcl::PointXYZ &in,
      pcl::PointXYZ &out,
      const double &angle,
      const double &origin_x,
      const double &origin_y) {
    double s = sin(angle);
    double c = cos(angle);

    out.x = c * (in.x - origin_x) - s * (in.y - origin_y) + origin_x;
    out.y = s * (in.x - origin_x) + c * (in.y - origin_y) + origin_y;
    out.z = in.z;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_road_segmentation = std::make_shared<RoadSegmentation>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_road_segmentation);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}