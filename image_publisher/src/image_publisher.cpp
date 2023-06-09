#include <image_publisher/image_publisher.hpp>

namespace license_detector {
ImagePublisher::ImagePublisher(const rclcpp::NodeOptions &options)
    : Node("image_publisher", options) {
  RCLCPP_INFO(this->get_logger(), "Start image_publisher_node");
  image_path = this->declare_parameter<std::string>("image_path", "");
  fps = this->declare_parameter<double>("fps", 10.0);
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(int(1000 / fps)),
                              std::bind(&ImagePublisher::publish_image, this));
}

void ImagePublisher::publish_image() {
  RCLCPP_DEBUG(get_logger(), "Publishing image");
  cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
  std_msgs::msg::Header header;
  header.stamp = this->now();
  auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  image_pub_->publish(*msg);
}
}  // namespace license_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(license_detector::ImagePublisher)