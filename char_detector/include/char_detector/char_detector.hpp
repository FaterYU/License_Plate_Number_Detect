#ifndef CHAR_DETECTOR__CHAR_DETECTOR_HPP_
#define CHAR_DETECTOR__CHAR_DETECTOR_HPP_

#include <cv_bridge/cv_bridge.h>

#include <char_detector/character_matcher.hpp>
#include <image_transport/image_transport.hpp>
#include <license_detector_interfaces/msg/license_char.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace license_detector {
class CharDetector : public rclcpp::Node {
 public:
  explicit CharDetector(const rclcpp::NodeOptions &options);

 private:
  rclcpp::Subscription<license_detector_interfaces::msg::LicenseChar>::SharedPtr
      license_char_image_sub_;
  void image_callback(
      const license_detector_interfaces::msg::LicenseChar::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr license_char_pub_;
  std::shared_ptr<CharacterMatcher> character_matcher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};
}  // namespace license_detector

#endif  // CHAR_DETECTOR__CHAR_DETECTOR_HPP_
