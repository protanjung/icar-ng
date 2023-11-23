#include "boost/thread/mutex.hpp"
#include "cv_bridge/cv_bridge.h"
#include "icar_interfaces/msg/lidar_rings.hpp"
#include "icar_interfaces/msg/pose.hpp"
#include "icar_interfaces/srv/px_cm_inference.hpp"
#include "opencv2/opencv.hpp"
#include "pandu_ros2_kit/help_marker.hpp"
#include "pcl/filters/passthrough.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#define CAMERA_ORIGIN_X 375
#define CAMERA_ORIGIN_Y 0

using namespace std::chrono_literals;

class RoadSegmentation : public rclcpp::Node {
 public:
  //-----Callback group
  rclcpp::CallbackGroup::SharedPtr cbg;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_10hz;
  //-----Susbcriber
  rclcpp::Subscription<icar_interfaces::msg::LidarRings>::SharedPtr sub_lidar_base_rings;
  rclcpp::Subscription<icar_interfaces::msg::Pose>::SharedPtr sub_pose;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_image_raw;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_raw;
  //-----Service client
  rclcpp::Client<icar_interfaces::srv::PxCmInference>::SharedPtr cli_pxcm_mlp;
  rclcpp::Client<icar_interfaces::srv::PxCmInference>::SharedPtr cli_cmpx_mlp;
  //-----Transform listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  //-----Mutex
  std::mutex mutex_color;
  std::mutex mutex_depth;
  //-----Help
  HelpMarker _marker;

  // Transform
  // =========
  bool tf_is_initialized = false;
  geometry_msgs::msg::TransformStamped tf_base_lidar_front;

  // Point cloud
  // ===========
  pcl::PointCloud<pcl::PointXYZ> cloud_left, cloud_pool_left[20], cloud_fit_cm_left, cloud_fit_px_left;
  pcl::PointCloud<pcl::PointXYZ> cloud_right, cloud_pool_right[20], cloud_fit_cm_right, cloud_fit_px_right;

  // Pose and twist
  // ==============
  geometry_msgs::msg::Pose2D pose;
  geometry_msgs::msg::Twist twist;

  // Image processing
  // ================
  cv::Mat mat_color = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
  cv::Mat mat_depth = cv::Mat::zeros(cv::Size(1280, 720), CV_16UC1);

  RoadSegmentation() : Node("road_segmentation") {
    //-----Callback group
    cbg = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    //-----Timer
    tim_10hz = this->create_wall_timer(100ms, std::bind(&RoadSegmentation::cllbck_tim_10hz, this), cbg);
    //-----Subscriber
    sub_lidar_base_rings = this->create_subscription<icar_interfaces::msg::LidarRings>(
        "lidar_base/rings", 10, std::bind(&RoadSegmentation::cllbck_sub_lidar_base_rings, this, std::placeholders::_1));
    sub_pose = this->create_subscription<icar_interfaces::msg::Pose>(
        "pose", 10, std::bind(&RoadSegmentation::cllbck_sub_pose, this, std::placeholders::_1));
    sub_color_image_raw = this->create_subscription<sensor_msgs::msg::Image>(
        "color/image_raw", 1, std::bind(&RoadSegmentation::cllbck_sub_color_image_raw, this, std::placeholders::_1));
    sub_depth_image_raw = this->create_subscription<sensor_msgs::msg::Image>(
        "aligned_depth_to_color/image_raw",
        1,
        std::bind(&RoadSegmentation::cllbck_sub_depth_image_raw, this, std::placeholders::_1));
    //-----Service client
    cli_pxcm_mlp =
        this->create_client<icar_interfaces::srv::PxCmInference>("pxcm_mlp", rmw_qos_profile_services_default, cbg);
    cli_cmpx_mlp =
        this->create_client<icar_interfaces::srv::PxCmInference>("cmpx_mlp", rmw_qos_profile_services_default, cbg);
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
    static const float origin_z = tf_base_lidar_front.transform.translation.z;

    // * Delta height
    // * ============
    pcl::PointCloud<pcl::PointXYZ> cloud_delta_height_left, cloud_delta_height_right;

    for (int i = 0; i < 6; i++) {
      pcl::PointCloud<pcl::PointXYZ> step0;
      pcl::PointCloud<pcl::PointXYZ> step1;
      pcl::PointXYZ step2_left, step2_right;
      pcl::PointXYZ step3_left, step3_right;

      ring_to_cloud(msg->ring[i], step0, origin_x, origin_y, origin_z);
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

      ring_to_cloud(msg->ring[i], step0, origin_x, origin_y, origin_z);
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

    // * Pooling
    // * =======
    bool is_new_pool = false;

    static geometry_msgs::msg::Pose2D last_pose;
    float dx = pose.x - last_pose.x;
    float dy = pose.y - last_pose.y;
    float r = sqrtf(dx * dx + dy * dy);
    if (r > 0.2) {
      /* The above code is calculating the difference in angle between two given angles, `a` and `o`. */
      float a = atan2f(dy, dx);
      float o = pose.theta;
      float delta_angle = a - o;
      if (delta_angle > M_PI) { delta_angle -= 2 * M_PI; }
      if (delta_angle < -M_PI) { delta_angle += 2 * M_PI; }

      bool is_forward = fabsf(delta_angle) < M_PI_2;

      /* Create transform matrix to be applied to the point cloud. */
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.translate(Eigen::Vector3f(is_forward ? -r : r, 0.0, 0.0));
      transform.rotate(Eigen::AngleAxisf(last_pose.theta - pose.theta, Eigen::Vector3f::UnitZ()));

      /* Apply transform matrix to the point cloud. */
      for (int i = 1; i < 20; i++) {
        pcl::transformPointCloud(cloud_pool_left[i], cloud_pool_left[i], transform);
        cloud_pool_left[i - 1] = cloud_pool_left[i];
        pcl::transformPointCloud(cloud_pool_right[i], cloud_pool_right[i], transform);
        cloud_pool_right[i - 1] = cloud_pool_right[i];
      }
      cloud_pool_left[19] = cloud_delta_height_left + cloud_delta_length_left + cloud_tangent_angle_left;
      cloud_pool_right[19] = cloud_delta_height_right + cloud_delta_length_right + cloud_tangent_angle_right;

      /* Pooling */
      cloud_left.clear();
      cloud_right.clear();
      for (int i = 0; i < 20; i++) {
        cloud_left += cloud_pool_left[i];
        cloud_right += cloud_pool_right[i];
      }

      /* Update last pose. */
      last_pose = pose;

      /* Set flag to true. */
      is_new_pool = true;
    }

    // * Fitting
    // * =======
    if (is_new_pool) {
      fit_cloud(cloud_left, 3.0, 12.0, 0.5, cloud_fit_cm_left);
      fit_cloud(cloud_right, 3.0, 12.0, 0.5, cloud_fit_cm_right);
    }

    // * CM to PX
    // * ========
    if (is_new_pool) {
      cloud_fit_px_left.clear();
      cloud_fit_px_right.clear();
      for (const auto &point : cloud_fit_cm_left.points) {
        float input[2] = {point.x * 100 - CAMERA_ORIGIN_X, point.y * 100 - CAMERA_ORIGIN_Y};
        float output[2];
        cmpx_mlp_call(input, output);

        pcl::PointXYZ p;
        p.x = output[0];
        p.y = output[1];
        p.z = 0;
        cloud_fit_px_left.push_back(p);
      }
      for (const auto &point : cloud_fit_cm_right.points) {
        float input[2] = {point.x * 100 - CAMERA_ORIGIN_X, point.y * 100 - CAMERA_ORIGIN_Y};
        float output[2];
        cmpx_mlp_call(input, output);

        pcl::PointXYZ p;
        p.x = output[0];
        p.y = output[1];
        p.z = 0;
        cloud_fit_px_right.push_back(p);
      }
    }

    // * Visualization
    // * =============
    if (is_new_pool) {
      draw_cloud("road_candidate_delta_height", 1, cloud_delta_height_left, {0.5, 0.5, 0.5, 1.0}, 0.1, 0.1);
      draw_cloud("road_candidate_delta_height", 2, cloud_delta_height_right, {1.0, 0.5, 0.5, 1.0}, 0.1, 0.1);
      draw_cloud("road_candidate_delta_length", 1, cloud_delta_length_left, {0.5, 0.0, 0.5, 1.0}, 0.1, 0.1);
      draw_cloud("road_candidate_delta_length", 2, cloud_delta_length_right, {1.0, 0.0, 0.5, 1.0}, 0.1, 0.1);
      draw_cloud("road_candidate_tangent_angle", 1, cloud_tangent_angle_left, {0.5, 0.5, 0.0, 1.0}, 0.1, 0.1);
      draw_cloud("road_candidate_tangent_angle", 2, cloud_tangent_angle_right, {1.0, 0.5, 0.0, 1.0}, 0.1, 0.1);
      draw_cloud("road_candidate", 1, cloud_left, {0.5, 0.6, 0.5, 1.0}, 0.1, 0.1);
      draw_cloud("road_candidate", 2, cloud_right, {1.0, 0.6, 0.5, 1.0}, 0.1, 0.1);
      draw_cloud("road_candidate_fit", 1, cloud_fit_cm_left, {0.5, 0.6, 1.0, 1.0}, 0.1, 0.1);
      draw_cloud("road_candidate_fit", 2, cloud_fit_cm_right, {1.0, 0.6, 1.0, 1.0}, 0.1, 0.1);

      mutex_color.lock();
      cv::Mat canvas = mat_color.clone();
      mutex_color.unlock();

      for (const auto &point : cloud_fit_px_left.points) {
        cv::circle(canvas, cv::Point(point.x, point.y), 10, cv::Scalar(127, 0, 255), -1);
      }
      for (const auto &point : cloud_fit_px_right.points) {
        cv::circle(canvas, cv::Point(point.x, point.y), 10, cv::Scalar(255, 127, 127), -1);
      }

      std::vector<cv::Point> polygon;
      polygon.push_back(cv::Point(0, 720));
      polygon.push_back(cv::Point(1280, 720));
      for (int i = 0; i < cloud_fit_px_right.points.size(); i++) {
        polygon.push_back(cv::Point(cloud_fit_px_right.points[i].x, cloud_fit_px_right.points[i].y));
      }
      for (int i = cloud_fit_px_left.points.size() - 1; i >= 0; i--) {
        polygon.push_back(cv::Point(cloud_fit_px_left.points[i].x, cloud_fit_px_left.points[i].y));
      }

      cv::Mat overlay = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
      cv::fillConvexPoly(overlay, polygon, cv::Scalar(0, 255, 0), cv::LINE_AA);
      cv::addWeighted(canvas, 0.75, overlay, 0.25, 0.0, canvas);

      cv::imshow("canvas", canvas);
      cv::waitKey(1);
    }
  }

  void cllbck_sub_pose(const icar_interfaces::msg::Pose::SharedPtr msg) {
    pose = msg->pose;
    twist = msg->twist;
  }

  void cllbck_sub_color_image_raw(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!mutex_color.try_lock()) { return; }
    mat_color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    mutex_color.unlock();
  }

  void cllbck_sub_depth_image_raw(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!mutex_depth.try_lock()) { return; }
    mat_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
    mutex_depth.unlock();
  }

  // ===================================

  bool road_segmentation_init() {
    while (!tf_is_initialized) {
      try {
        tf_base_lidar_front = tf_buffer->lookupTransform("base_link", "lidar_front_link", tf2::TimePointZero);
        tf_is_initialized = true;
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "TF lookup timeout, it is normal if it happens at startup. Retrying..");
        std::this_thread::sleep_for(1s);
      }
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
    for (float y = 0.0; y < 5.0; y += 0.2) {
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
    for (float y = 0.0; y > -5.0; y -= 0.2) {
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
    for (float y = 0.0; y < 5.0; y += 0.2) {
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
    for (float y = 0.0; y > -5.0; y -= 0.2) {
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
    for (int i = index_offset + 48; i < 88; i++) {
      float tangent0_x = in.point[i - 2].x;
      float tangent0_y = in.point[i - 2].y;
      float tangent1_x = in.point[i + 2].x;
      float tangent1_y = in.point[i + 2].y;
      float radial_x = in.point[i].x;
      float radial_y = in.point[i].y;

      float angle_tangent = atan2f(tangent1_y - tangent0_y, tangent1_x - tangent0_x);
      float angle_radial = atan2f(radial_y - origin_y, radial_x - origin_x);

      if (cosf(angle_tangent - angle_radial) > 0.5 || cosf(angle_tangent - angle_radial) < -0.5) {
        out_left.x = radial_x;
        out_left.y = radial_y;
        out_left.z = in.point[i].z;
        break;
      }
    }

    // * Right side
    // * ==========
    for (int i = index_offset + 42; i > 2; i--) {
      float tangent0_x = in.point[i - 2].x;
      float tangent0_y = in.point[i - 2].y;
      float tangent1_x = in.point[i + 2].x;
      float tangent1_y = in.point[i + 2].y;
      float radial_x = in.point[i].x;
      float radial_y = in.point[i].y;

      float angle_tangent = atan2f(tangent1_y - tangent0_y, tangent1_x - tangent0_x);
      float angle_radial = atan2f(radial_y - origin_y, radial_x - origin_x);

      if (cosf(angle_tangent - angle_radial) > 0.5 || cosf(angle_tangent - angle_radial) < -0.5) {
        out_right.x = radial_x;
        out_right.y = radial_y;
        out_right.z = in.point[i].z;
        break;
      }
    }
  }

  // ===================================

  void ring_to_cloud(
      const icar_interfaces::msg::LidarRing &in,
      pcl::PointCloud<pcl::PointXYZ> &out,
      float origin_x,
      float origin_y,
      float origin_z) {
    out.clear();
    for (const auto &point : in.point) {
      float dx = point.x - origin_x;
      float dy = point.y - origin_y;
      float dz = point.z - origin_z;
      if (sqrtf(dx * dx + dy * dy + dz * dz) > 0.1) {
        pcl::PointXYZ p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        out.push_back(p);
      }
    }
  }

  void fit_cloud(
      const pcl::PointCloud<pcl::PointXYZ> &in,
      float min_x,
      float max_x,
      float step_x,
      pcl::PointCloud<pcl::PointXYZ> &out) {
    int n = in.points.size();

    std::vector<double> x(n);
    std::vector<double> y(n);
    for (int i = 0; i < n; i++) {
      x[i] = in.points[i].x;
      y[i] = in.points[i].y;
    }
    Eigen::Map<Eigen::VectorXd> x_eigen(x.data(), n);
    Eigen::Map<Eigen::VectorXd> y_eigen(y.data(), n);

    Eigen::MatrixXd X(n, 3);
    X.col(0) = Eigen::VectorXd::Ones(n);
    X.col(1) = x_eigen;
    X.col(2) = (x_eigen.array().square());

    Eigen::Vector3d coeffs = (X.transpose() * X).ldlt().solve(X.transpose() * y_eigen);

    float a = coeffs(0);
    float b = coeffs(1);
    float c = coeffs(2);

    out.clear();
    for (float x = min_x; x < max_x; x += step_x) {
      pcl::PointXYZ p;
      p.x = x;
      p.y = a + b * x + c * x * x;
      p.z = 0;
      out.push_back(p);
    }
  }

  void draw_cloud(
      const std::string &ns,
      const int &id,
      const pcl::PointCloud<pcl::PointXYZ> &in,
      const std::vector<float> &color,
      const float &scale_width,
      const float &scale_height) {
    std::vector<geometry_msgs::msg::Point> points;
    for (const auto &point : in.points) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      points.push_back(p);
    }

    if (points.empty()) {
      _marker.points("base_link", ns, -id, points, color, scale_width, scale_height);
    } else {
      _marker.points("base_link", ns, id, points, color, scale_width, scale_height);
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

  // ===================================

  void pxcm_mlp_call(float *input, float *output) {
    if (cli_pxcm_mlp->wait_for_service(1s) == false) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to pxcm_mlp");
      return;
    }

    auto request = std::make_shared<icar_interfaces::srv::PxCmInference::Request>();
    request->x = input[0];
    request->y = input[1];

    auto result = cli_pxcm_mlp->async_send_request(request);
    result.wait();

    auto response = result.get();
    output[0] = response->x;
    output[1] = response->y;
  }

  void cmpx_mlp_call(float *input, float *output) {
    if (cli_cmpx_mlp->wait_for_service(1s) == false) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to cmpx_mlp");
      return;
    }

    auto request = std::make_shared<icar_interfaces::srv::PxCmInference::Request>();
    request->x = input[0];
    request->y = input[1];

    auto result = cli_cmpx_mlp->async_send_request(request);
    result.wait();

    auto response = result.get();
    output[0] = response->x;
    output[1] = response->y;
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