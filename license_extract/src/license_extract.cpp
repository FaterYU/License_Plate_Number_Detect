#include <license_extract/license_extract.hpp>

namespace license_detector {
LicenseExtract::LicenseExtract(const rclcpp::NodeOptions &options)
    : Node("license_extract", options) {
  RCLCPP_INFO(this->get_logger(), "Start license_extract_node");
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10,
      std::bind(&LicenseExtract::image_callback, this, std::placeholders::_1));
  blue_extractor_ = std::make_shared<Extractor>(
      cv::Scalar(0, 150, 0), cv::Scalar(255, 255, 255),
      cv::Scalar(100, 43, 120), cv::Scalar(124, 255, 255));
  debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/extract/debug_image", 10);
  license_char_image_pub_ =
      this->create_publisher<license_detector_interfaces::msg::LicenseChar>(
          "license_char_image", 10);
}

void LicenseExtract::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  // RCLCPP_DEBUG(this->get_logger(), "Received image");
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

  std_msgs::msg::Header header;
  header.stamp = msg->header.stamp;
  // publish license char image
  license_detector_interfaces::msg::LicenseChar license_char;
  for (auto &i : result) {
    auto msg = cv_bridge::CvImage(header, "mono8", i).toImageMsg();
    license_char.license_image.push_back(*msg);
  }
  license_char_image_pub_->publish(license_char);
  RCLCPP_DEBUG(this->get_logger(), "Publish license char image");

  // get debug binary image
  cv::Mat debug_image = blue_extractor_->get_debug_image();

  auto debug_msg =
      cv_bridge::CvImage(header, "mono8", debug_image).toImageMsg();
  debug_image_pub_->publish(*debug_msg);
  RCLCPP_DEBUG(this->get_logger(), "Publish debug image");
}

}  // namespace license_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(license_detector::LicenseExtract)