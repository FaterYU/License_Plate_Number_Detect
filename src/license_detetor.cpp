#include <license_detetor/license_detetor.hpp>

namespace license_detetor {
LicenseDetetor::LicenseDetetor(const rclcpp::NodeOptions &options)
    : Node("license_detetor", options) {
  RCLCPP_INFO(this->get_logger(), "Start license_detetor_node");
  image_path = this->declare_parameter<std::string>("image_path", "");
  fps = this->declare_parameter<double>("fps", 10.0);
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(int(1000 / fps)),
                              std::bind(&LicenseDetetor::publish_image, this));
}

void LicenseDetetor::publish_image() {
  RCLCPP_INFO(get_logger(), "Publishing image");
  cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
  auto msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
  image_pub_->publish(*msg);
}
}  // namespace license_detetor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(license_detetor::LicenseDetetor)
