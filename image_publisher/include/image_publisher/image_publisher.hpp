#ifndef IMAGE_PUBLISHER__IMAGE_PUBLISHER_HPP_
#define IMAGE_PUBLISHER__IMAGE_PUBLISHER_HPP_

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace license_detetor {
class ImagePublisher : public rclcpp::Node {
 public:
  explicit ImagePublisher(const rclcpp::NodeOptions &options);

 private:
  void publish_image();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string image_path;
  double fps;
};
}  // namespace license_detetor

#endif  // IMAGE_PUBLISHER__IMAGE_PUBLISHER_HPP_
