#include <chrono>
#include <fstream>
#include <mutex>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class CameraDatasetLabeler : public rclcpp::Node {
   private:
    //-----Timer
    rclcpp::TimerBase::SharedPtr tim_30hz;
    //-----Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_image_raw;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_raw;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_filtered;
    //-----Mutex
    std::mutex mtx_color;
    std::mutex mtx_depth;

    // Misc
    // ====
    std::string config_dir = std::getenv("HOME") + std::string("/icar-ng-data/config");
    std::string dataset_dir = std::getenv("HOME") + std::string("/icar-ng-data/dataset");

    // Image processing
    // ================
    cv::Mat mat_color = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
    cv::Mat mat_depth = cv::Mat::zeros(cv::Size(1280, 720), CV_16UC1);

    std::vector<cv::Point> lowest_points_of_contour;
    std::vector<cv::Point> selected_points_of_contour;

    int h_min = 0, h_max = 180;
    int s_min = 0, s_max = 255;
    int v_min = 0, v_max = 255;

    // IMU
    // ===
    double roll = 0, pitch = 0, yaw = 0;

   public:
    CameraDatasetLabeler() : Node("camera_dataset_labeler") {
        //-----Timer
        tim_30hz = this->create_wall_timer(33ms, std::bind(&CameraDatasetLabeler::cllbck_tim_30hz, this));
        //-----Subscriber
        sub_color_image_raw = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", 1,
            std::bind(&CameraDatasetLabeler::cllbck_sub_color_image_raw, this, std::placeholders::_1));
        sub_depth_image_raw = this->create_subscription<sensor_msgs::msg::Image>(
            "/aligned_depth_to_color/image_raw", 1,
            std::bind(&CameraDatasetLabeler::cllbck_sub_depth_image_raw, this, std::placeholders::_1));
        sub_imu_filtered = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_filtered", 1, std::bind(&CameraDatasetLabeler::cllbck_sub_imu_filtered, this, std::placeholders::_1));

        /* The below code is creating two directories using the `mkdir` command in C++. The first
        directory is created using the `config_dir` variable, and the second directory is created
        using the `dataset_dir` variable. The `mkdir -p` command ensures that the directories are
        created recursively, meaning that any parent directories that do not exist will also be
        created. */
        system(("mkdir -p " + config_dir).c_str());
        system(("mkdir -p " + dataset_dir).c_str());

        /* The below code is loading a configuration file or settings for HSV thresholding. The
        `load_config()` function is defined below. */
        load_config();

        /* The below code is creating a graphical user interface (GUI) for a camera dataset labeler application
        using the OpenCV library in C++. */
        cv::namedWindow("Camera Dataset Labeler", cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(
            "Camera Dataset Labeler",
            [](int event, int x, int y, int flags, void *userdata) {
                (void)flags;
                CameraDatasetLabeler *node = reinterpret_cast<CameraDatasetLabeler *>(userdata);

                /* The below code is multiplying the variables x and y by 2 and storing the results in the variables x_
                and y_. */
                int x_ = x * 2;
                int y_ = y * 2;

                /* The below code is checking if the values of `x_` and `y_` are within the specified range. If either
                `x_` is less than 0, greater than or equal to 1280, `y_` is less than 0, or greater than or equal to
                720, the code will return and exit the current function. */
                if (x_ < 0 || x_ >= 1280 || y_ < 0 || y_ >= 720) {
                    return;
                }

                /* The below code is checking if the mouse button is pressed down (specifically the middle button)
                during an event. If the condition is true, it iterates through a list of points called
                "lowest_points_of_contour" and checks if the distance between the mouse coordinates (x_, y_) and each
                point is less than 10. If the condition is true, the point is added to another list called
                "selected_points_of_contour" and a warning message is logged with the coordinates of the added point.
                The loop breaks after adding the first point that satisfies the condition. */
                if (event == cv::EVENT_MBUTTONDOWN) {
                    for (auto point : node->lowest_points_of_contour) {
                        if (node->distance(x_, y_, point.x, point.y) < 10) {
                            node->selected_points_of_contour.push_back(point);
                            RCLCPP_WARN(node->get_logger(), "Added (%d, %d)", point.x, point.y);
                            break;
                        }
                    }
                }
                /* The below code checks if the event is a left button down event (cv::EVENT_LBUTTONDOWN). If the
                condition is true, it adds a cv::Point with the coordinates (x_, y_) to the selected_points_of_contour
                vector of the node object. It also logs a warning message using RCLCPP_WARN, indicating that the point
                has been added. */
                else if (event == cv::EVENT_LBUTTONDOWN) {
                    node->selected_points_of_contour.push_back(cv::Point(x_, y_));
                    RCLCPP_WARN(node->get_logger(), "Added (%d, %d)", x_, y_);
                }
                /* The below code is checking if the right mouse button is clicked (event == cv::EVENT_RBUTTONDOWN). If
                it is, it checks if there are any selected points of a contour in the node. If there are, it removes the
                last selected point from the list (node->selected_points_of_contour.pop_back()) and logs a warning
                message ("Removed") using the RCLCPP_WARN macro. */
                else if (event == cv::EVENT_RBUTTONDOWN) {
                    if (node->selected_points_of_contour.size() > 0) {
                        node->selected_points_of_contour.pop_back();
                        RCLCPP_WARN(node->get_logger(), "Removed");
                    }
                }
            },
            this);
        cv::createTrackbar(
            "H min", "Camera Dataset Labeler", NULL, 180,
            [](int value, void *userdata) {
                CameraDatasetLabeler *node = reinterpret_cast<CameraDatasetLabeler *>(userdata);
                node->h_min = value;
            },
            this);
        cv::createTrackbar(
            "H max", "Camera Dataset Labeler", NULL, 180,
            [](int value, void *userdata) {
                CameraDatasetLabeler *node = reinterpret_cast<CameraDatasetLabeler *>(userdata);
                node->h_max = value;
            },
            this);
        cv::createTrackbar(
            "S min", "Camera Dataset Labeler", NULL, 255,
            [](int value, void *userdata) {
                CameraDatasetLabeler *node = reinterpret_cast<CameraDatasetLabeler *>(userdata);
                node->s_min = value;
            },
            this);
        cv::createTrackbar(
            "S max", "Camera Dataset Labeler", NULL, 255,
            [](int value, void *userdata) {
                CameraDatasetLabeler *node = reinterpret_cast<CameraDatasetLabeler *>(userdata);
                node->s_max = value;
            },
            this);
        cv::createTrackbar(
            "V min", "Camera Dataset Labeler", NULL, 255,
            [](int value, void *userdata) {
                CameraDatasetLabeler *node = reinterpret_cast<CameraDatasetLabeler *>(userdata);
                node->v_min = value;
            },
            this);
        cv::createTrackbar(
            "V max", "Camera Dataset Labeler", NULL, 255,
            [](int value, void *userdata) {
                CameraDatasetLabeler *node = reinterpret_cast<CameraDatasetLabeler *>(userdata);
                node->v_max = value;
            },
            this);
        cv::setTrackbarPos("H min", "Camera Dataset Labeler", h_min);
        cv::setTrackbarPos("H max", "Camera Dataset Labeler", h_max);
        cv::setTrackbarPos("S min", "Camera Dataset Labeler", s_min);
        cv::setTrackbarPos("S max", "Camera Dataset Labeler", s_max);
        cv::setTrackbarPos("V min", "Camera Dataset Labeler", v_min);
        cv::setTrackbarPos("V max", "Camera Dataset Labeler", v_max);
    }

    //==================================

    void cllbck_tim_30hz() {
        mtx_color.lock();
        cv::Mat mat_bgr = mat_color.clone();
        mtx_color.unlock();

        /* Draw alignment line and zero marker. This will help aligning the checkerboard with the camera. */
        cv::line(mat_bgr, cv::Point(640, 0), cv::Point(640, 720), cv::Scalar(0, 0, 255), 2);
        cv::circle(mat_bgr, cv::Point(640, 690), 5, cv::Scalar(0, 0, 255), 2);

        /* The below code is converting an image from the BGR color space to the HSV color space using OpenCV
        in C++. */
        cv::Mat mat_hsv;
        cv::cvtColor(mat_bgr, mat_hsv, cv::COLOR_BGR2HSV);

        /* The below code is performing color thresholding on an input image in the HSV color space. It creates
        a binary mask image, where pixels within a specified range of HSV values are set to white (255) and
        pixels outside the range are set to black (0). */
        cv::Mat mat_mask;
        if (h_min < h_max) {
            cv::inRange(mat_hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mat_mask);
        } else {
            cv::Mat mat_mask1, mat_mask2;
            cv::inRange(mat_hsv, cv::Scalar(0, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mat_mask1);
            cv::inRange(mat_hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(180, s_max, v_max), mat_mask2);
            cv::bitwise_or(mat_mask1, mat_mask2, mat_mask);
        }

        /* The below code is performing morphological operations on an input image represented by the matrix
        `mat_mask`. */
        cv::morphologyEx(mat_mask, mat_mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::morphologyEx(mat_mask, mat_mask, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        /* The below code is finding contours in an image using OpenCV in C++. It takes an input image
        `mat_mask` and stores the contours in the `contours` vector. The `hierarchy` vector stores the
        hierarchical relationships between contours. The function `cv::findContours` is called with the
        parameters `cv::RETR_TREE` and `cv::CHAIN_APPROX_SIMPLE` to specify the retrieval mode and contour
        approximation method respectively. */
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mat_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        /* The below code is drawing contours on an image called "mat_bgr". The contours are colored red
        (0, 0, 255) and have a thickness of 2 pixels. */
        cv::drawContours(mat_bgr, contours, -1, cv::Scalar(0, 0, 255), 2);

        /* The below code is iterating through a vector of contours and collecting the center and bottom point
        of each contour. The center point is the average of all the points in the contour, and the bottom point
        is the point with the largest y-coordinate. The center and bottom points of each contour are stored in
        the `lowest_points_of_contour` vector. */
        std::vector<cv::Point> lowest_points_of_contour_temp;
        for (size_t i = 0; i < contours.size(); i++) {
            // Check if the contour is a child contour.
            if (hierarchy[i][3] != -1) {
                continue;
            }

            // Get the center and bottom point of the contour.
            int center_x = 0, center_y = 0, bottom_y = 0;
            for (size_t j = 0; j < contours[i].size(); j++) {
                center_x += contours[i][j].x;
                center_y += contours[i][j].y;
                bottom_y = std::max(bottom_y, contours[i][j].y);
            }
            center_x /= contours[i].size();
            center_y /= contours[i].size();

            // Append the center and bottom point of the contour to the vector.
            lowest_points_of_contour_temp.push_back(cv::Point(center_x, bottom_y));

            // Draw the center and bottom point of the contour.
            cv::circle(mat_bgr, cv::Point(center_x, center_y), 5, cv::Scalar(0, 0, 255), 2);
            cv::circle(mat_bgr, cv::Point(center_x, bottom_y), 5, cv::Scalar(0, 255, 0), 2);
        }
        lowest_points_of_contour = lowest_points_of_contour_temp;

        /* The below code is iterating over a collection of points called "selected_points_of_contour" and
        drawing circles on an image called "mat_bgr" at each of these points. The circles have a radius of 3
        pixels, are colored blue (255, 0, 0), and have a thickness of 2 pixels. */
        for (auto point : selected_points_of_contour) {
            cv::circle(mat_bgr, point, 3, cv::Scalar(255, 0, 0), 2);
        }

        /* Preparing the image for display. */
        cv::Mat mat_bgr_small;
        cv::resize(mat_bgr, mat_bgr_small, cv::Size(640, 360));
        cv::putText(mat_bgr_small, "Roll: " + std::to_string(roll * 180 / M_PI), cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        cv::putText(mat_bgr_small, "Pitch: " + std::to_string(pitch * 180 / M_PI), cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

        /* Preparing the image for display. */
        cv::Mat mat_mask_small;
        cv::resize(mat_mask, mat_mask_small, cv::Size(640, 360));
        cv::cvtColor(mat_mask_small, mat_mask_small, cv::COLOR_GRAY2BGR);

        /* Preparing the image for display. */
        cv::Mat mat_display;
        cv::hconcat(mat_bgr_small, mat_mask_small, mat_display);

        /* Displaying the image. */
        cv::imshow("Camera Dataset Labeler", mat_display);

        char c = cv::waitKey(1);

        if (c == 'q') {
            save_config();
            RCLCPP_WARN(this->get_logger(), "Saved config");
        } else if (c == 's') {
            save_dataset();
            RCLCPP_WARN(this->get_logger(), "Saved dataset");
        }

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

    void load_config() {
        std::ifstream ifs(config_dir + std::string("/hsv_threshold.txt"));
        if (ifs.is_open()) {
            ifs >> h_min >> h_max >> s_min >> s_max >> v_min >> v_max;
            ifs.close();
        }
    }

    void save_config() {
        std::ofstream ofs(config_dir + std::string("/hsv_threshold.txt"));
        if (ofs.is_open()) {
            ofs << h_min << " " << h_max << " " << s_min << " " << s_max << " " << v_min << " " << v_max;
            ofs.close();
        }
    }

    void save_dataset() {
        std::ofstream dataset_file;

        std::time_t time_ = std::time(nullptr);
        std::tm tm_ = *std::localtime(&time_);
        char timestamp[20];
        std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d-%H-%M-%S", &tm_);

        std::string dataset_latest = std::string("/dataset_latest.csv");
        dataset_file.open(dataset_dir + dataset_latest, std::ofstream::out | std::ofstream::trunc);
        if (dataset_file.is_open()) {
            dataset_file << "x,y" << std::endl;
            for (auto point : selected_points_of_contour) {
                dataset_file << point.x << "," << point.y << std::endl;
            }
            dataset_file.close();
        }

        std::string dataset_timestamp = std::string("/dataset_") + std::string(timestamp) + std::string(".csv");
        dataset_file.open(dataset_dir + dataset_timestamp, std::ofstream::out | std::ofstream::trunc);
        if (dataset_file.is_open()) {
            dataset_file << "x,y" << std::endl;
            for (auto point : selected_points_of_contour) {
                dataset_file << point.x << "," << point.y << std::endl;
            }
            dataset_file.close();
        }
    }

    float distance(float x0, float y0, float x1, float y1) {
        return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraDatasetLabeler>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}