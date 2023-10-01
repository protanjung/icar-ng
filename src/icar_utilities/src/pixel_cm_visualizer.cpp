#include <chrono>
#include <mutex>

#include "cv_bridge/cv_bridge.h"
#include "icar_interfaces/srv/pixel_cm_inference.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class PixelCMVisualizer : public rclcpp::Node {
   private:
    //-----Timer
    rclcpp::TimerBase::SharedPtr tim_30hz;
    //-----Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_image_raw;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_raw;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_filtered;
    //-----Service client
    rclcpp::Client<icar_interfaces::srv::PixelCMInference>::SharedPtr cli_pixel2cm_mlp;
    rclcpp::Client<icar_interfaces::srv::PixelCMInference>::SharedPtr cli_cm2pixel_mlp;

    //-----Mutex
    std::mutex mtx_color;
    std::mutex mtx_depth;

    // Image processing
    // ================
    cv::Mat mat_color = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
    cv::Mat mat_depth = cv::Mat::zeros(cv::Size(1280, 720), CV_16UC1);

    int horizon = 120;

    // IMU
    // ===
    double roll = 0, pitch = 0, yaw = 0;

    // Pixel and cm data
    // =================
    float pixel_x = 0, pixel_y = 0;
    float cm_x = 0, cm_y = 0;
    bool pixel_or_cm = false;

   public:
    PixelCMVisualizer() : Node("pixel_cm_visualizer") {
        //-----Timer
        tim_30hz = this->create_wall_timer(33ms, std::bind(&PixelCMVisualizer::cllbck_tim_30hz, this));
        //-----Subscriber
        sub_color_image_raw = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", 1,
            std::bind(&PixelCMVisualizer::cllbck_sub_color_image_raw, this, std::placeholders::_1));
        sub_depth_image_raw = this->create_subscription<sensor_msgs::msg::Image>(
            "/aligned_depth_to_color/image_raw", 1,
            std::bind(&PixelCMVisualizer::cllbck_sub_depth_image_raw, this, std::placeholders::_1));
        sub_imu_filtered = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_filtered", 1, std::bind(&PixelCMVisualizer::cllbck_sub_imu_filtered, this, std::placeholders::_1));
        //-----Service client
        cli_pixel2cm_mlp = this->create_client<icar_interfaces::srv::PixelCMInference>("/pixel2cm_mlp");
        cli_cm2pixel_mlp = this->create_client<icar_interfaces::srv::PixelCMInference>("/cm2pixel_mlp");

        cv::namedWindow("Pixel Visualizer", cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(
            "Pixel Visualizer",
            [](int event, int x, int y, int flags, void *userdata) {
                (void)event;
                (void)flags;
                PixelCMVisualizer *node = reinterpret_cast<PixelCMVisualizer *>(userdata);

                /* The below code is multiplying the variables x and y by 2 and storing the results in the variables x_
                and y_. */
                int x_ = x * 2;
                int y_ = y * 2;

                /* The below code is checking if the values of `x_` and `y_` are within the specified range. If either
                `x_` is less than 0, greater than or equal to 1280, `y_` is less than `node->horizon`, or greater than
                or equal to 720, then the function returns. */
                if (x_ < 0 || x_ >= 1280 || y_ < node->horizon || y_ >= 720) {
                    return;
                }

                /* The code is updating the `pixel_x` and `pixel_y` variables of the `PixelCMVisualizer` class with the
                coordinates of the selected pixel. It also sets the `pixel_or_cm` flag to true, indicating that the
                user has selected a pixel rather than a centimeter on the image. */
                node->pixel_x = x_;
                node->pixel_y = y_;
                node->pixel_or_cm = true;
            },
            this);
        cv::createTrackbar(
            "Horizon", "Pixel Visualizer", NULL, 720,
            [](int value, void *userdata) {
                PixelCMVisualizer *node = reinterpret_cast<PixelCMVisualizer *>(userdata);
                node->horizon = value;
            },
            this);
        cv::setTrackbarPos("Horizon", "Pixel Visualizer", horizon);

        cv::namedWindow("CM Visualizer", cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(
            "CM Visualizer",
            [](int event, int x, int y, int flags, void *userdata) {
                (void)event;
                (void)flags;
                PixelCMVisualizer *node = reinterpret_cast<PixelCMVisualizer *>(userdata);

                cv::Size area = cv::Size(1000, 800);
                cv::Point offset = cv::Point(170, 400);

                /* The below code is multiplying the variables x and y by 2 and storing the results in the variables
                x_ and y_. */
                int x_ = x * 2;
                int y_ = y * 2;

                /* Rotate the coordinates by 90 degrees. */
                int x__ = -y_ + area.width;
                int y__ = x_;

                /* The above code is converting pixel coordinates (x__, y__) to centimeter coordinates using the
                cm_pixel2cm function of the cv::Point class. The resulting centimeter coordinates are then assigned
                to the node's cm_x and cm_y variables. The pixel_or_cm variable is set to false, indicating that the
                coordinates are now in centimeters. */
                cv::Point cm = node->cm_pixel2cm(cv::Point(x__, y__), area, offset);
                node->cm_x = cm.x;
                node->cm_y = cm.y;
                node->pixel_or_cm = false;
            },
            this);
    }

    //==================================

    void cllbck_tim_30hz() {
        if (pixel_or_cm) {
            if (!cli_pixel2cm_mlp->wait_for_service(1s)) {
                RCLCPP_ERROR(this->get_logger(), "Service not available");
                return;
            }

            auto request = std::make_shared<icar_interfaces::srv::PixelCMInference::Request>();
            request->x_in = pixel_x;
            request->y_in = pixel_y;

            auto result = cli_pixel2cm_mlp->async_send_request(
                request, std::bind(&PixelCMVisualizer::cllbck_cli_pixel2cm_mlp, this, std::placeholders::_1));
        } else {
            if (!cli_cm2pixel_mlp->wait_for_service(1s)) {
                RCLCPP_ERROR(this->get_logger(), "Service not available");
                return;
            }

            auto request = std::make_shared<icar_interfaces::srv::PixelCMInference::Request>();
            request->x_in = cm_x;
            request->y_in = cm_y;

            auto result = cli_cm2pixel_mlp->async_send_request(
                request, std::bind(&PixelCMVisualizer::cllbck_cli_cm2pixel_mlp, this, std::placeholders::_1));
        }

        // =============================

        mtx_color.lock();
        cv::Mat mat_bgr = mat_color.clone();
        mtx_color.unlock();

        /* Draw alignment line and zero marker. This will help aligning the checkerboard with the camera. */
        cv::line(mat_bgr, cv::Point(640, 0), cv::Point(640, 720), cv::Scalar(0, 0, 255), 2);
        cv::circle(mat_bgr, cv::Point(640, 690), 5, cv::Scalar(0, 0, 255), 2);

        /* Draw horizon line. The horizon is limiting the area so that the user can only select pixels in the lower
        half of the image. */
        cv::line(mat_bgr, cv::Point(0, horizon), cv::Point(1280, horizon), cv::Scalar(0, 255, 0), 2);

        /* Draw crosshair on the selected pixel. */
        cv::Point pixel = cv::Point(pixel_x, pixel_y);
        cv::line(mat_bgr, pixel + cv::Point(-10, 0), pixel + cv::Point(10, 0), cv::Scalar(255, 0, 0), 2);
        cv::line(mat_bgr, pixel + cv::Point(0, -10), pixel + cv::Point(0, 10), cv::Scalar(255, 0, 0), 2);

        cv::Mat mat_cm;
        cm_init(mat_cm);
        cm_draw(mat_cm);

        /* Preparing the image for display. */
        cv::Mat mat_bgr_small;
        cv::resize(mat_bgr, mat_bgr_small, cv::Size(), 0.5, 0.5);
        cv::line(mat_bgr_small, cv::Point(320, 0), cv::Point(320, 360), cv::Scalar(0, 0, 255), 1);
        cv::circle(mat_bgr_small, cv::Point(320, 240), 3, cv::Scalar(0, 0, 255), 1);
        cv::putText(mat_bgr_small, "X: " + std::to_string((int)pixel_x), cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(0, 0, 255), 1);
        cv::putText(mat_bgr_small, "Y: " + std::to_string((int)pixel_y), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(0, 0, 255), 1);
        cv::putText(mat_bgr_small, "Roll: " + std::to_string(roll * 180 / M_PI), cv::Point(100, 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
        cv::putText(mat_bgr_small, "Pitch: " + std::to_string(pitch * 180 / M_PI), cv::Point(100, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);

        /* Preparing the image for display. */
        cv::Mat mat_cm_small;
        cv::resize(mat_cm, mat_cm_small, cv::Size(), 0.5, 0.5);
        cv::rotate(mat_cm_small, mat_cm_small, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::putText(mat_cm_small, "X: " + std::to_string((int)cm_x), cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(0, 0, 255), 1);
        cv::putText(mat_cm_small, "Y: " + std::to_string((int)cm_y), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(0, 0, 255), 1);
        cv::putText(mat_cm_small, "Roll: " + std::to_string(roll * 180 / M_PI), cv::Point(100, 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
        cv::putText(mat_cm_small, "Pitch: " + std::to_string(pitch * 180 / M_PI), cv::Point(100, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);

        /* Displaying the image. */
        cv::imshow("Pixel Visualizer", mat_bgr_small);
        cv::imshow("CM Visualizer", mat_cm_small);

        char c = cv::waitKey(1);

        if (c == 'q') {
            rclcpp::shutdown();
        }
    }

    //==================================

    void cllbck_sub_color_image_raw(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mtx_color.try_lock()) {
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            mat_color = cv_ptr->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        mtx_color.unlock();
    }

    void cllbck_sub_depth_image_raw(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!mtx_depth.try_lock()) {
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            mat_depth = cv_ptr->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        mtx_depth.unlock();
    }

    void cllbck_sub_imu_filtered(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }

    //==================================

    void cllbck_cli_pixel2cm_mlp(rclcpp::Client<icar_interfaces::srv::PixelCMInference>::SharedFuture future) {
        auto result = future.get();
        cm_x = result->x_out;
        cm_y = result->y_out;
    }

    void cllbck_cli_cm2pixel_mlp(rclcpp::Client<icar_interfaces::srv::PixelCMInference>::SharedFuture future) {
        auto result = future.get();
        pixel_x = result->x_out;
        pixel_y = result->y_out;
    }

    //==================================

    cv::Point cm_cm2pixel(const cv::Point &input, cv::Size area = cv::Size(1000, 1000),
                          cv::Point offset = cv::Point(500, 500)) {
        cv::Point output;
        output.x = offset.x + input.x;
        output.y = area.height - (offset.y + input.y);
        return output;
    }

    cv::Point cm_pixel2cm(const cv::Point &input, cv::Size area = cv::Size(1000, 1000),
                          cv::Point offset = cv::Point(500, 500)) {
        cv::Point output;
        output.x = input.x - offset.x;
        output.y = -input.y + area.height - offset.y;
        return output;
    }

    void cm_init(cv::Mat &mat_cm) {
        cv::Size area = cv::Size(1000, 800);
        cv::Point offset = cv::Point(170, 400);

        int min_x = -offset.x + 50;
        int max_x = area.width - offset.x - 50;
        int min_y = -offset.y + 50;
        int max_y = area.height - offset.y - 50;

        mat_cm = cv::Mat::zeros(area, CV_8UC3);

        int row = 0;
        for (int x = 0; x < max_x; x += 50) {
            bool white = row % 2 == 0;
            for (int y = 0; y < max_y; y += 50) {
                if (white) {
                    cv::rectangle(
                        mat_cm, cm_cm2pixel(cv::Point(x, y), area, offset),
                        cm_cm2pixel(cv::Point(std::min(x + 50, max_x), std::min(y + 50, max_y)), area, offset),
                        cv::Scalar(63, 63, 63), -1);
                }
                white = !white;
            }
            white = row % 2 == 1;
            for (int y = 0; y > min_y; y -= 50) {
                if (white) {
                    cv::rectangle(
                        mat_cm, cm_cm2pixel(cv::Point(x, y), area, offset),
                        cm_cm2pixel(cv::Point(std::min(x + 50, max_x), std::max(y - 50, min_y)), area, offset),
                        cv::Scalar(63, 63, 63), -1);
                }
                white = !white;
            }
            row++;
        }
        row = 0;
        for (int x = 0; x > min_x; x -= 50) {
            bool white = row % 2 == 1;
            for (int y = 0; y < max_y; y += 50) {
                if (white) {
                    cv::rectangle(
                        mat_cm, cm_cm2pixel(cv::Point(x, y), area, offset),
                        cm_cm2pixel(cv::Point(std::max(x - 50, min_x), std::min(y + 50, max_y)), area, offset),
                        cv::Scalar(63, 63, 63), -1);
                }
                white = !white;
            }
            white = row % 2 == 0;
            for (int y = 0; y > min_y; y -= 50) {
                if (white) {
                    cv::rectangle(
                        mat_cm, cm_cm2pixel(cv::Point(x, y), area, offset),
                        cm_cm2pixel(cv::Point(std::max(x - 50, min_x), std::max(y - 50, min_y)), area, offset),
                        cv::Scalar(63, 63, 63), -1);
                }
                white = !white;
            }
            row++;
        }

        cv::rectangle(mat_cm, cm_cm2pixel(cv::Point(min_x, min_y), area, offset),
                      cm_cm2pixel(cv::Point(max_x, max_y), area, offset), cv::Scalar(255, 255, 255), 2);

        cv::line(mat_cm, cm_cm2pixel(cv::Point(0, 0), area, offset), cm_cm2pixel(cv::Point(50, 0), area, offset),
                 cv::Scalar(0, 0, 255), 2);
        cv::line(mat_cm, cm_cm2pixel(cv::Point(0, 0), area, offset), cm_cm2pixel(cv::Point(0, 50), area, offset),
                 cv::Scalar(0, 255, 0), 2);
    }

    void cm_draw(cv::Mat &mat_cm) {
        cv::Size area = cv::Size(1000, 800);
        cv::Point offset = cv::Point(170, 400);

        /* Draw crosshair on the selected pixel. */
        cv::Point pixel = cm_cm2pixel(cv::Point(cm_x, cm_y), area, offset);
        cv::line(mat_cm, pixel + cv::Point(-10, 0), pixel + cv::Point(10, 0), cv::Scalar(255, 0, 0), 2);
        cv::line(mat_cm, pixel + cv::Point(0, -10), pixel + cv::Point(0, 10), cv::Scalar(255, 0, 0), 2);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PixelCMVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}