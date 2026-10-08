#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class PubCenterFrame : public rclcpp::Node
{
public:
  PubCenterFrame() : Node("pub_center_frame")
  {
    // Declare parameters with default values
    this->declare_parameter<int>("cicle_radius", 10);
    // OpenCV scalar for BGR color: (B, G, R) -> Red is (0, 0, 255)
    this->declare_parameter<int>("circle_thickness", -1);

    this->radius = this->get_parameter("cicle_radius").as_int();
    this->thickness = this->get_parameter("circle_thickness").as_int();

    // Setup subscribers and publishers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&PubCenterFrame::procedure, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/center_frame", 10);
    center_pub_ = this->create_publisher<std_msgs::msg::String>("/center_frame_coor", 10);

    center_ = std::make_pair(0, 0);
  }

private:
  void procedure(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received image");

    cv::Mat cv_image;
    try
    {
      cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    int height = cv_image.rows;
    int width = cv_image.cols;

    center_ = std::make_pair(width / 2, height / 2);
    RCLCPP_INFO(this->get_logger(), "Center: (%d, %d)", center_.first, center_.second);

    // Draw circle (BGR format: Blue=0, Green=0, Red=255)
    cv::circle(cv_image, cv::Point(center_.first, center_.second), radius, cv::Scalar(0, 0, 255), thickness);

    publish_center_frame();

    // Publish processed image
    image_pub_->publish(*(cv_bridge::CvImage(msg->header, "bgr8", cv_image).toImageMsg()));
    RCLCPP_INFO(this->get_logger(), "Published center frame and image");
  }

  void publish_center_frame()
  {
    auto center_frame_msg = std_msgs::msg::String();
    center_frame_msg.data = std::to_string(center_.first) + ", " + std::to_string(center_.second);
    center_pub_->publish(center_frame_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr center_pub_;

  int radius;
  int thickness;
  std::pair<int, int> center_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PubCenterFrame>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}