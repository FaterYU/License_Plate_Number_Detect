#ifndef CHAR_DETECTOR__CHARACTER_MATCHER_HPP_
#define CHAR_DETECTOR__CHARACTER_MATCHER_HPP_

#include <cv_bridge/cv_bridge.h>

#include <license_detector_interfaces/msg/license_char.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace license_detector {
class CharacterMatcher {
 public:
  explicit CharacterMatcher();

  std::string match_character(
      const license_detector_interfaces::msg::LicenseChar::SharedPtr msg);
  cv::Mat debug_img;

 private:
  std::vector<cv::Mat> license_plate_sig_bin;
  void font_read();
  cv::Mat del_white(cv::Mat img);

  std::vector<cv::Mat> font_ch_;
  std::vector<cv::Mat> font_word_;
  std::vector<cv::Mat> font_num_;
  std::vector<std::string> font_ch_name_;
  std::vector<std::string> font_word_name_;
  std::vector<std::string> font_num_name_;
};
}  // namespace license_detector

#endif  // CHAR_DETECTOR__CHARACTER_MATCHER_HPP_