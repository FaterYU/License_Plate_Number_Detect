#include <char_detector/char_detector.hpp>

namespace license_detector {
CharDetector::CharDetector(const rclcpp::NodeOptions &options)
    : Node("char_detector", options) {
  RCLCPP_INFO(this->get_logger(), "Start char_detector_node");
  license_char_image_sub_ =
      this->create_subscription<license_detector_interfaces::msg::LicenseChar>(
          "license_char_image", 10,
          std::bind(&CharDetector::image_callback, this,
                    std::placeholders::_1));
  license_char_pub_ =
      this->create_publisher<std_msgs::msg::String>("license_char", 10);
  character_matcher_ = std::make_shared<CharacterMatcher>();
  debug_image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("/char/debug_image", 10);
}

void CharDetector::image_callback(
    const license_detector_interfaces::msg::LicenseChar::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Receive license_char_image");
  std_msgs::msg::String license_char_msg;
  license_char_msg.data = character_matcher_->match_character(msg);
  license_char_pub_->publish(license_char_msg);

  cv::Mat debug_image_cv = character_matcher_->debug_img;
  auto debug_image_ptr =
      cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", debug_image_cv)
          .toImageMsg();
  debug_image_pub_->publish(*debug_image_ptr);
}

}  // namespace license_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(license_detector::CharDetector)
