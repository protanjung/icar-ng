#include "tf2_ros/transform_broadcaster.h"

#include "icar_interfaces/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class TransformBroadcaster : public rclcpp::Node {
 public:
  //-----Parameter
  std::vector<double> icar_tf_rear_axle;
  std::vector<double> icar_tf_front_axle;
  std::vector<double> icar_tf_gps;
  std::vector<double> icar_tf_body;
  std::vector<double> icar_tf_camera;
  std::vector<double> icar_tf_lidar_front;
  std::vector<double> icar_tf_lidar_rearright;
  //-----Timer
  rclcpp::TimerBase::SharedPtr tim_100hz;
  //-----Subscriber
  rclcpp::Subscription<icar_interfaces::msg::Pose>::SharedPtr sub_pose;
  //-----Transform broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  TransformBroadcaster() : Node("transform_broadcaster") {
    //-----Parameter
    this->declare_parameter("icar.tf.rear_axle", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("icar.tf.front_axle", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("icar.tf.gps", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("icar.tf.body", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("icar.tf.camera", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("icar.tf.lidar_front", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("icar.tf.lidar_rearright", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->get_parameter("icar.tf.rear_axle", icar_tf_rear_axle);
    this->get_parameter("icar.tf.front_axle", icar_tf_front_axle);
    this->get_parameter("icar.tf.gps", icar_tf_gps);
    this->get_parameter("icar.tf.body", icar_tf_body);
    this->get_parameter("icar.tf.camera", icar_tf_camera);
    this->get_parameter("icar.tf.lidar_front", icar_tf_lidar_front);
    this->get_parameter("icar.tf.lidar_rearright", icar_tf_lidar_rearright);
    //-----Timer
    tim_100hz = this->create_wall_timer(10ms, std::bind(&TransformBroadcaster::cllbck_tim_100hz, this));
    //-----Subscriber
    sub_pose = this->create_subscription<icar_interfaces::msg::Pose>(
        "pose", 10, std::bind(&TransformBroadcaster::cllbck_sub_pose, this, std::placeholders::_1));
    //-----Transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    if (transform_broadcaster_init() == false) {
      RCLCPP_ERROR(this->get_logger(), "Transform broadcaster init failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_tim_100hz() {
    if (transform_broadcaster_routine() == false) {
      RCLCPP_ERROR(this->get_logger(), "Transform broadcaster routine failed");
      rclcpp::shutdown();
    }
  }

  //====================================

  void cllbck_sub_pose(const icar_interfaces::msg::Pose::SharedPtr msg) {
    send_transform(
        std::vector<double>{msg->pose.x, msg->pose.y, 0, 0, 0, msg->pose.theta * 180 / M_PI}, "map", "base_link");
  }

  //====================================

  bool transform_broadcaster_init() {
    send_static_transform(icar_tf_rear_axle, "base_link", "rear_axle_link");
    send_static_transform(icar_tf_front_axle, "base_link", "front_axle_link");
    send_static_transform(icar_tf_gps, "base_link", "gps_link");
    send_static_transform(icar_tf_body, "base_link", "body_link");
    send_static_transform(icar_tf_camera, "base_link", "camera_link");
    send_static_transform(icar_tf_lidar_front, "base_link", "lidar_front_link");
    send_static_transform(icar_tf_lidar_rearright, "base_link", "lidar_rearright_link");

    return true;
  }

  bool transform_broadcaster_routine() { return true; }

  //====================================

  void send_transform(std::vector<double> tf, std::string frame_id, std::string child_frame_id) {
    if (tf.size() != 6) { return; }

    tf2::Vector3 origin;
    origin.setValue(tf[0], tf[1], tf[2]);

    tf2::Quaternion rotation;
    rotation.setRPY(tf[3] * M_PI / 180, tf[4] * M_PI / 180, tf[5] * M_PI / 180);

    geometry_msgs::msg::TransformStamped transform_stamped;
    // ----
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    // ----
    transform_stamped.transform.translation.x = origin.x();
    transform_stamped.transform.translation.y = origin.y();
    transform_stamped.transform.translation.z = origin.z();
    // ----
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();
    // ----
    tf_broadcaster->sendTransform(transform_stamped);
  }

  void send_static_transform(std::vector<double> tf, std::string frame_id, std::string child_frame_id) {
    if (tf.size() != 6) { return; }

    tf2::Vector3 origin;
    origin.setValue(tf[0], tf[1], tf[2]);

    tf2::Quaternion rotation;
    rotation.setRPY(tf[3] * M_PI / 180, tf[4] * M_PI / 180, tf[5] * M_PI / 180);

    geometry_msgs::msg::TransformStamped transform_stamped;
    // ----
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    // ----
    transform_stamped.transform.translation.x = origin.x();
    transform_stamped.transform.translation.y = origin.y();
    transform_stamped.transform.translation.z = origin.z();
    // ----
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();
    // ----
    static_tf_broadcaster->sendTransform(transform_stamped);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_transform_broadcaster = std::make_shared<TransformBroadcaster>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_transform_broadcaster);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}