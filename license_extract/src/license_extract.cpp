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
      cv::Scalar(100, 43, 120), cv::Scalar(124, 255, 255), "white");
  green_extractor_ = std::make_shared<Extractor>(
      cv::Scalar(0, 0, 0), cv::Scalar(255, 50, 255), cv::Scalar(35, 43, 46),
      cv::Scalar(99, 255, 255), "black");
  yellow_extractor_ = std::make_shared<Extractor>(
      cv::Scalar(0, 0, 0), cv::Scalar(255, 50, 255), cv::Scalar(0, 114, 100),
      cv::Scalar(40, 240, 255), "black");

  debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/extract/debug_image", 10);
  license_char_image_pub_ =
      this->create_publisher<license_detector_interfaces::msg::LicenseChar>(
          "license_char_image", 10);
  license_type_pub_ =
      this->create_publisher<std_msgs::msg::String>("license_type", 10);
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

  std::string text_color = "undefined";
  std::vector<cv::Mat> result;
  cv::Mat debug_image;
  std_msgs::msg::Header header;
  header.stamp = msg->header.stamp;

  // image process
  std::vector<cv::Mat> blue_result = blue_extractor_->image_process(image);
  std::vector<cv::Mat> green_result = green_extractor_->image_process(image);
  std::vector<cv::Mat> yellow_result = yellow_extractor_->image_process(image);

  std_msgs::msg::String license_type_msg;
  if (blue_result.size() > 0) {
    text_color = "white";
    result = blue_result;
    debug_image = blue_extractor_->get_debug_image();
    license_type_msg.data = "蓝色普通车牌";
  } else if (green_result.size() > 0) {
    text_color = "black";
    result = green_result;
    debug_image = green_extractor_->get_debug_image();
    license_type_msg.data = "绿色新能源车牌";
  } else if (yellow_result.size() > 0) {
    text_color = "black";
    result = yellow_result;
    debug_image = yellow_extractor_->get_debug_image();
    license_type_msg.data = "黄色普通车牌";
  } else {
    RCLCPP_DEBUG(this->get_logger(), "No license plate");
    license_type_msg.data = "未检出车牌";
  }
  license_type_pub_->publish(license_type_msg);

  // debug_image = green_extractor_->get_debug_image();
  if (result.size() == 0) {
    RCLCPP_DEBUG(this->get_logger(), "No license char image");
    return;
  }

  // publish debug image
  auto debug_msg =
      cv_bridge::CvImage(header, "mono8", debug_image).toImageMsg();
  debug_image_pub_->publish(*debug_msg);
  RCLCPP_DEBUG(this->get_logger(), "Publish debug image");

  // publish license char image
  license_detector_interfaces::msg::LicenseChar license_char;
  for (auto &i : result) {
    auto msg = cv_bridge::CvImage(header, "mono8", i).toImageMsg();
    license_char.license_image.push_back(*msg);
  }
  license_char.license_plate_text_color = text_color;
  license_char_image_pub_->publish(license_char);
  RCLCPP_DEBUG(this->get_logger(), "Publish license char image");
}

}  // namespace license_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(license_detector::LicenseExtract)