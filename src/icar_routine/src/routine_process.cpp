#include "icar_routine/routine.hpp"

void Routine::process_metric() {}

void Routine::process_marker() {
  {
    geometry_msgs::msg::Point p;

    static const float tyre_diameter = icar_tyre_rim * 0.0254 + icar_tyre_width * icar_tyre_aspect_ratio * 0.00002;
    static const float tyre_width = icar_tyre_width * 0.001;

    _marker.cube(
        "body_link",
        "body",
        1,
        p,
        rpy_to_quaternion(0, 0, 0),
        std::vector<float>{1.0, 1.0, 1.0, 0.5},
        icar_body_length,
        icar_body_width,
        icar_body_height);

    p.y = icar_body_width / 2 - tyre_width / 2;
    _marker.cylinder(
        "front_axle_link",
        "tyre",
        1,
        p,
        rpy_to_quaternion(M_PI_2, 0, 0),
        std::vector<float>{0.0, 0.0, 0.0, 0.5},
        tyre_diameter,
        tyre_diameter,
        tyre_width);
    _marker.cylinder(
        "rear_axle_link",
        "tyre",
        2,
        p,
        rpy_to_quaternion(M_PI_2, 0, 0),
        std::vector<float>{0.0, 0.0, 0.0, 0.5},
        tyre_diameter,
        tyre_diameter,
        tyre_width);

    p.y = -icar_body_width / 2 + tyre_width / 2;
    _marker.cylinder(
        "front_axle_link",
        "tyre",
        3,
        p,
        rpy_to_quaternion(M_PI_2, 0, 0),
        std::vector<float>{0.0, 0.0, 0.0, 0.5},
        tyre_diameter,
        tyre_diameter,
        tyre_width);
    _marker.cylinder(
        "rear_axle_link",
        "tyre",
        4,
        p,
        rpy_to_quaternion(M_PI_2, 0, 0),
        std::vector<float>{0.0, 0.0, 0.0, 0.5},
        tyre_diameter,
        tyre_diameter,
        tyre_width);
  }
}

//====================================

geometry_msgs::msg::Quaternion Routine::rpy_to_quaternion(float roll, float pitch, float yaw) {
  tf2::Quaternion q_in;
  geometry_msgs::msg::Quaternion q_out;

  q_in.setRPY(roll, pitch, yaw);
  tf2::convert(q_in, q_out);

  return q_out;
}
