#include <license_extract/license_extract.hpp>

namespace license_detetor {
LicenseExtract::LicenseExtract(const rclcpp::NodeOptions &options)
    : Node("license_extract", options) {
  RCLCPP_INFO(this->get_logger(), "Start license_extract_node");
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10,
      std::bind(&LicenseExtract::image_callback, this, std::placeholders::_1));
  blue_extractor_ = std::make_shared<Extractor>(
      cv::Scalar(0, 150, 0), cv::Scalar(255, 255, 255),
      cv::Scalar(100, 43, 120), cv::Scalar(124, 255, 255));
  debug_image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
}

void LicenseExtract::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received image");
  // to cv::Mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image = cv_ptr->image;

  // image process
  std::vector<cv::Mat> result = blue_extractor_->image_process(image);
  RCLCPP_INFO(this->get_logger(), "Result size: %ld", result.size());
  // get debug binary image
  cv::Mat debug_image = result[0];

  auto debug_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", debug_image)
          .toImageMsg();
  debug_image_pub_->publish(*debug_msg);
  RCLCPP_INFO(this->get_logger(), "Publish debug image");
}

}  // namespace license_detetor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(license_detetor::LicenseExtract)