#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector()
        : Node("aruco_detector")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10,
            std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));

        dictionary_ =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        RCLCPP_INFO(this->get_logger(), "Aruco Detector Started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(
            cv_ptr->image,
            dictionary_,
            corners,
            ids);

        if (!ids.empty())
        {
            for (int id : ids)
            {
                RCLCPP_INFO(this->get_logger(), "Marker ID : %d", id);
            }
        }

        cv::aruco::drawDetectedMarkers(
            cv_ptr->image,
            corners,
            ids);

        cv::imshow("aruco", cv_ptr->image);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ArucoDetector>());

    rclcpp::shutdown();

    return 0;
}
