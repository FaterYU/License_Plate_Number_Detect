#ifndef EXTRACTOR__EXTRACTOR_HPP_
#define EXTRACTOR__EXTRACTOR_HPP_

#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace license_detector {
class Extractor {
 public:
  explicit Extractor(cv::Scalar background_low, cv::Scalar background_high,
                     cv::Scalar text_low, cv::Scalar text_high);
  std::vector<cv::Mat> image_process(const cv::Mat &image);
  cv::Mat get_debug_image();

 private:
  cv::Scalar background_low_;
  cv::Scalar background_high_;
  cv::Scalar text_low_;
  cv::Scalar text_high_;

  cv::Mat origin_image_;

  cv::Mat background_mask_;
  cv::Mat text_mask_;

  cv::Mat aim_mask_;
  cv::Mat aim_image_;
  cv::Mat aim_image_gray_;
  cv::Mat aim_image_binary_;

  cv::Mat background_image_;
  cv::Mat background_image_gray_;
  cv::Mat background_image_binary_;

  cv::Mat text_image_;
  cv::Mat text_image_gray_;
  cv::Mat text_image_binary_;

  std::vector<cv::Mat> result_bgr_list_;
  std::vector<cv::Mat> licence_plate_sig_bin_;

  cv::Mat debug_image_;

  void image_convert(const cv::Mat &image);
  void find_contours();
  bool is_license_plate(const cv::Mat &image);
  void get_char_image_list();
  void reset();
};
}  // namespace license_detector

#endif  // EXTRACTOR__EXTRACTOR_HPP_
