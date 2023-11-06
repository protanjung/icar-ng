#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "icar_interfaces/msg/gps_to_pc.hpp"
#include "icar_interfaces/msg/pose.hpp"
#include "icar_interfaces/msg/stm32_to_pc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#define RADIUS(b, a) sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2))
#define ANGLE(b, a) atan2(b.y - a.y, b.x - a.x)

using namespace std::chrono_literals;

class PoseEstimator : public rclcpp::Node {
 public:
  //-----Parameter
  double icar_conversion_encoder_pulse_to_meter;
  double icar_cf_alpha_xy;
  double icar_cf_alpha_theta;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_100hz;
  //-----Subscriber
  rclcpp::Subscription<icar_interfaces::msg::GpsToPc>::SharedPtr sub_gps_to_pc;
  rclcpp::Subscription<icar_interfaces::msg::Stm32ToPc>::SharedPtr sub_stm32_to_pc;
  //-----Publisher
  rclcpp::Publisher<icar_interfaces::msg::Pose>::SharedPtr pub_pose;
  //-----Transform listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;

  // Transform
  // =========
  bool tf_is_initialized = false;
  geometry_msgs::msg::TransformStamped tf_gps_base;

  // Encoder dan gyroscope
  // =====================
  uint16_t encoder_kiri;
  uint16_t encoder_kanan;
  float gyroscope;

  // Global positioning system
  // =========================
  bool gps_is_fix;

  // Pose and twist
  // ==============
  geometry_msgs::msg::Pose2D gps_pose2d;
  geometry_msgs::msg::Twist odometry_twist;
  geometry_msgs::msg::Pose2D odometry_pose2d;
  geometry_msgs::msg::Twist pose_twist;
  geometry_msgs::msg::Pose2D pose_pose2d;

  PoseEstimator() : Node("pose_estimator") {
    //-----Parameter
    this->declare_parameter("icar.conversion.encoder_pulse_to_meter", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("icar.cf.alpha_xy", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("icar.cf.alpha_theta", rclcpp::PARAMETER_DOUBLE);
    this->get_parameter("icar.conversion.encoder_pulse_to_meter", icar_conversion_encoder_pulse_to_meter);
    this->get_parameter("icar.cf.alpha_xy", icar_cf_alpha_xy);
    this->get_parameter("icar.cf.alpha_theta", icar_cf_alpha_theta);
    //-----Timer
    tim_100hz = this->create_wall_timer(10ms, std::bind(&PoseEstimator::cllbck_tim_100hz, this));
    //-----Subscriber
    sub_gps_to_pc = this->create_subscription<icar_interfaces::msg::GpsToPc>(
        "gps/to_pc", 10, std::bind(&PoseEstimator::cllbck_sub_gps_to_pc, this, std::placeholders::_1));
    sub_stm32_to_pc = this->create_subscription<icar_interfaces::msg::Stm32ToPc>(
        "stm32/to_pc", 10, std::bind(&PoseEstimator::cllbck_sub_stm32_to_pc, this, std::placeholders::_1));
    //-----Publisher
    pub_pose = this->create_publisher<icar_interfaces::msg::Pose>("pose", 10);
    //-----Transform listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    if (pose_estimator_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Pose estimator init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_100hz() {
    if (pose_estimator_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Pose estimator routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_gps_to_pc(const icar_interfaces::msg::GpsToPc::SharedPtr msg) {
    if (!tf_is_initialized) { return; }

    /* The above code is checking if the GPS fix status is set to "fix" in a ROS message of type
    sensor_msgs::msg::NavSatFix. It assigns the result of the comparison to the variable gps_is_fix. */
    gps_is_fix = msg->navsatfix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX;

    /* The above code is calculating the distance and angle between two points in a 2D space. */
    float r = sqrtf(powf(tf_gps_base.transform.translation.x, 2) + powf(tf_gps_base.transform.translation.y, 2));
    float a = atan2f(tf_gps_base.transform.translation.y, tf_gps_base.transform.translation.x);

    /* The above code is calculating the GPS position (x, y, and theta) based on the current position (x,
    y, and theta) and a given distance (r) and angle (a). It uses trigonometric functions to calculate
    the new x and y coordinates based on the current position and the given distance and angle. The
    theta value remains the same as the current position. */
    gps_pose2d.x = msg->pose2d.x + r * cosf(a + msg->pose2d.theta);
    gps_pose2d.y = msg->pose2d.y + r * sinf(a + msg->pose2d.theta);
    gps_pose2d.theta = msg->pose2d.theta;
  }

  void cllbck_sub_stm32_to_pc(const icar_interfaces::msg::Stm32ToPc::SharedPtr msg) {
    encoder_kiri = msg->encoder_kiri;
    encoder_kanan = msg->encoder_kanan;
    gyroscope = msg->gyroscope;

    // =================================

    static rclcpp::Time last_time = this->now();
    static rclcpp::Time current_time = this->now();
    last_time = current_time;
    current_time = this->now();
    double dt = (current_time - last_time).seconds();

    // =================================

    int16_t d_encoder_kiri, d_encoder_kanan;
    delta_encoder(encoder_kiri, encoder_kanan, d_encoder_kiri, d_encoder_kanan);
    if (abs(d_encoder_kiri) > 1024 || abs(d_encoder_kanan) > 1024) {
      RCLCPP_ERROR(this->get_logger(), "Delta encoder value is too big (%d, %d)", d_encoder_kiri, d_encoder_kanan);
      return;
    }

    float d_gyroscope;
    delta_gyroscope(gyroscope, d_gyroscope);
    if (abs(d_gyroscope) > 90) {
      RCLCPP_ERROR(this->get_logger(), "Delta gyroscope value is too big (%f)", d_gyroscope);
      return;
    }

    // =================================

    /* This code is calculating translation component of the odometry. */
    odometry_pose2d.x +=
        (d_encoder_kiri + d_encoder_kanan) / 2 * cos(odometry_pose2d.theta) * icar_conversion_encoder_pulse_to_meter;
    odometry_pose2d.y +=
        (d_encoder_kiri + d_encoder_kanan) / 2 * sin(odometry_pose2d.theta) * icar_conversion_encoder_pulse_to_meter;
    odometry_twist.linear.x = (d_encoder_kiri + d_encoder_kanan) / 2 * icar_conversion_encoder_pulse_to_meter / dt;

    /* This code is calculating rotation component of the odometry. */
    odometry_pose2d.theta += d_gyroscope * M_PI / 180;
    if (odometry_pose2d.theta > M_PI) {
      odometry_pose2d.theta -= 2 * M_PI;
    } else if (odometry_pose2d.theta < -M_PI) {
      odometry_pose2d.theta += 2 * M_PI;
    }
    odometry_twist.angular.z = d_gyroscope * M_PI / 180 / dt;

    // =================================

    /* This code is calculating translation component of the pose. */
    pose_pose2d.x += odometry_twist.linear.x * cos(pose_pose2d.theta) * dt;
    pose_pose2d.y += odometry_twist.linear.x * sin(pose_pose2d.theta) * dt;
    pose_twist.linear.x = odometry_twist.linear.x;

    /* This code is calculating rotation component of the pose. */
    pose_pose2d.theta += odometry_twist.angular.z * dt;
    if (pose_pose2d.theta > M_PI) {
      pose_pose2d.theta -= 2 * M_PI;
    } else if (pose_pose2d.theta < -M_PI) {
      pose_pose2d.theta += 2 * M_PI;
    }
    pose_twist.angular.z = odometry_twist.angular.z;
  }

  //====================================

  bool pose_estimator_init() {
    RCLCPP_INFO(this->get_logger(), "Encoder pulse to meter: %f", icar_conversion_encoder_pulse_to_meter);
    RCLCPP_INFO(this->get_logger(), "Alpha XY: %f", icar_cf_alpha_xy);
    RCLCPP_INFO(this->get_logger(), "Alpha Theta: %f", icar_cf_alpha_theta);

    while (!tf_is_initialized) {
      try {
        tf_gps_base = tf_buffer->lookupTransform("gps_link", "base_link", tf2::TimePointZero);
        tf_is_initialized = true;
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "TF lookup timeout, it is normal if it happens at startup. Retrying..");
        std::this_thread::sleep_for(1s);
      }
    }

    return true;
  }

  bool pose_estimator_routine() {
    fuse_xy();
    fuse_theta();

    icar_interfaces::msg::Pose msg_pose;
    msg_pose.pose = pose_pose2d;
    msg_pose.twist = pose_twist;
    pub_pose->publish(msg_pose);

    return true;
  }

  //====================================

  void delta_encoder(uint16_t encoder_kiri, uint16_t encoder_kanan, int16_t &d_encoder_kiri, int16_t &d_encoder_kanan) {
    static bool is_initialized = false;
    static uint16_t last_encoder_kiri = 0;
    static uint16_t last_encoder_kanan = 0;

    if (is_initialized == false && (last_encoder_kiri == 0 || last_encoder_kanan == 0)) {
      last_encoder_kiri = encoder_kiri;
      last_encoder_kanan = encoder_kanan;
      is_initialized = true;
      return;
    }

    if (encoder_kiri < 1024 && last_encoder_kiri > 3072) {
      d_encoder_kiri = encoder_kiri - last_encoder_kiri + 4096;
    } else if (encoder_kiri > 3072 && last_encoder_kiri < 1024) {
      d_encoder_kiri = encoder_kiri - last_encoder_kiri - 4096;
    } else {
      d_encoder_kiri = encoder_kiri - last_encoder_kiri;
    }

    if (encoder_kanan < 1024 && last_encoder_kanan > 3072) {
      d_encoder_kanan = encoder_kanan - last_encoder_kanan + 4096;
    } else if (encoder_kanan > 3072 && last_encoder_kanan < 1024) {
      d_encoder_kanan = encoder_kanan - last_encoder_kanan - 4096;
    } else {
      d_encoder_kanan = encoder_kanan - last_encoder_kanan;
    }

    last_encoder_kiri = encoder_kiri;
    last_encoder_kanan = encoder_kanan;
  }

  void delta_gyroscope(float gyroscope, float &d_gyroscope) {
    static bool is_initialized = false;
    static float last_gyroscope = 0;

    if (is_initialized == false && last_gyroscope == 0) {
      last_gyroscope = gyroscope;
      is_initialized = true;
      return;
    }

    if (gyroscope < -90 && last_gyroscope > 90) {
      d_gyroscope = gyroscope - last_gyroscope + 360;
    } else if (gyroscope > 90 && last_gyroscope < -90) {
      d_gyroscope = gyroscope - last_gyroscope - 360;
    } else {
      d_gyroscope = gyroscope - last_gyroscope;
    }

    last_gyroscope = gyroscope;
  }

  //====================================

  void fuse_xy() {
    static uint8_t init_counter = 0;

    static geometry_msgs::msg::Pose2D last_odometry_pose2d;
    static geometry_msgs::msg::Pose2D last_gps_pose2d;
    if (last_odometry_pose2d.x == 0 || last_odometry_pose2d.y == 0) { last_odometry_pose2d = odometry_pose2d; }
    if (last_gps_pose2d.x == 0 || last_gps_pose2d.y == 0) { last_gps_pose2d = gps_pose2d; }

    if (RADIUS(odometry_pose2d, last_odometry_pose2d) > 0.1) {
      if (gps_is_fix) {
        float delta_x = gps_pose2d.x - pose_pose2d.x;
        float delta_y = gps_pose2d.y - pose_pose2d.y;

        if (init_counter >= 50) {
          pose_pose2d.x = (icar_cf_alpha_xy) * (pose_pose2d.x) + (1 - icar_cf_alpha_xy) * (pose_pose2d.x + delta_x);
          pose_pose2d.y = (icar_cf_alpha_xy) * (pose_pose2d.y) + (1 - icar_cf_alpha_xy) * (pose_pose2d.y + delta_y);
        } else {
          pose_pose2d.x = (0.2) * (pose_pose2d.x) + (1 - 0.2) * (pose_pose2d.x + delta_x);
          pose_pose2d.y = (0.2) * (pose_pose2d.y) + (1 - 0.2) * (pose_pose2d.y + delta_y);

          if (++init_counter >= 50) { RCLCPP_WARN(this->get_logger(), "XY pose estimator init done"); }
        }
      }

      last_odometry_pose2d = odometry_pose2d;
      last_gps_pose2d = gps_pose2d;
    }
  }

  void fuse_theta() {
    static uint8_t init_counter = 0;

    static geometry_msgs::msg::Pose2D last_odometry_pose2d;
    static geometry_msgs::msg::Pose2D last_gps_pose2d;
    if (last_odometry_pose2d.x == 0 || last_odometry_pose2d.y == 0) { last_odometry_pose2d = odometry_pose2d; }
    if (last_gps_pose2d.x == 0 || last_gps_pose2d.y == 0) { last_gps_pose2d = gps_pose2d; }

    if (RADIUS(odometry_pose2d, last_odometry_pose2d) > 1.0) {
      if (gps_is_fix) {
        float delta_angle = gps_pose2d.theta - pose_pose2d.theta;
        if (delta_angle > M_PI) {
          delta_angle -= 2 * M_PI;
        } else if (delta_angle < -M_PI) {
          delta_angle += 2 * M_PI;
        }

        float delta_angle_corrected = fabsf(delta_angle) < M_PI_2 ? delta_angle : delta_angle + M_PI;
        if (delta_angle_corrected > M_PI) {
          delta_angle_corrected -= 2 * M_PI;
        } else if (delta_angle_corrected < -M_PI) {
          delta_angle_corrected += 2 * M_PI;
        }

        if (init_counter >= 10) {
          pose_pose2d.theta = (icar_cf_alpha_theta) * (pose_pose2d.theta) +
                              (1 - icar_cf_alpha_theta) * (pose_pose2d.theta + delta_angle_corrected);
          if (pose_pose2d.theta > M_PI) {
            pose_pose2d.theta -= 2 * M_PI;
          } else if (pose_pose2d.theta < -M_PI) {
            pose_pose2d.theta += 2 * M_PI;
          }
        } else {
          pose_pose2d.theta = (0.2) * (pose_pose2d.theta) + (1 - 0.2) * (pose_pose2d.theta + delta_angle);
          if (pose_pose2d.theta > M_PI) {
            pose_pose2d.theta -= 2 * M_PI;
          } else if (pose_pose2d.theta < -M_PI) {
            pose_pose2d.theta += 2 * M_PI;
          }

          if (++init_counter >= 10) { RCLCPP_WARN(this->get_logger(), "Theta pose estimator init done"); }
        }
      }

      last_odometry_pose2d = odometry_pose2d;
      last_gps_pose2d = gps_pose2d;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_pose_estimator = std::make_shared<PoseEstimator>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_pose_estimator);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}