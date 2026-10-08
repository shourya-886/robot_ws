#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ColorToGrey : public rclcpp::Node
{
public:
  ColorToGrey() : Node("pub_center_frame")
  {
    // Initialize subscriber and publisher
    color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&ColorToGrey::color_to_grey_callback, this, std::placeholders::_1));

    grey_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_grey", 10);
  }

private:
  void color_to_grey_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received color image");

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

    cv::Mat grey_image;
    cv::cvtColor(cv_image, grey_image, cv::COLOR_BGR2GRAY);

    // Publish the grayscale image with mono8 encoding
    grey_image_pub_->publish(*(cv_bridge::CvImage(msg->header, "mono8", grey_image).toImageMsg()));
    RCLCPP_INFO(this->get_logger(), "Published grey image");
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr grey_image_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorToGrey>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}