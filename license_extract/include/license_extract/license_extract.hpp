#ifndef LICENSE_EXTRACT__LICENSE_EXTRACT_HPP_
#define LICENSE_EXTRACT__LICENSE_EXTRACT_HPP_

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <license_detector_interfaces/msg/license_char.hpp>
#include <license_extract/extractor.hpp>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace license_detector {
class LicenseExtract : public rclcpp::Node {
 public:
  explicit LicenseExtract(const rclcpp::NodeOptions &options);

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  std::shared_ptr<Extractor> blue_extractor_;
  std::shared_ptr<Extractor> green_extractor_;
  std::shared_ptr<Extractor> yellow_extractor_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<license_detector_interfaces::msg::LicenseChar>::SharedPtr
      license_char_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr license_type_pub_;
};
}  // namespace license_detector

#endif  // LICENSE_EXTRACT__LICENSE_EXTRACT_HPP_