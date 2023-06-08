#include <license_extract/extractor.hpp>

namespace license_detector {
Extractor::Extractor(cv::Scalar text_low, cv::Scalar text_high,
                     cv::Scalar background_low, cv::Scalar background_high,
                     std::string text_color)
    : text_low_(text_low),
      text_high_(text_high),
      background_low_(background_low),
      background_high_(background_high),
      text_color_(text_color) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Start Extractor");
}

cv::Mat Extractor::get_debug_image() {
  // cv::Mat debug_image = background_image_binary_;
  // debug_image_ = debug_image;
  return debug_image_;
}

std::vector<cv::Mat> Extractor::image_process(const cv::Mat &image) {
  reset();
  image_convert(image);
  find_contours();
  get_char_image_list();
  return licence_plate_sig_bin_;
}

void Extractor::image_convert(const cv::Mat &image) {
  cv::Mat hls_image;
  cv::Mat hsv_image;
  origin_image_ = image.clone();
  cv::cvtColor(image, hls_image, cv::COLOR_BGR2HLS);
  cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
  cv::inRange(hls_image, text_low_, text_high_, text_mask_);
  cv::inRange(hsv_image, background_low_, background_high_, background_mask_);

  cv::Mat add_mask;
  cv::Mat sub_mask;
  cv::add(text_mask_, background_mask_, add_mask);
  cv::subtract(text_mask_, background_mask_, sub_mask);
  cv::subtract(add_mask, sub_mask, aim_mask_);

  cv::bitwise_and(image, image, aim_image_, aim_mask_);
  cv::cvtColor(aim_image_, aim_image_gray_, cv::COLOR_BGR2GRAY);
  cv::adaptiveThreshold(aim_image_gray_, aim_image_binary_, 255,
                        cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 3,
                        2);
  double threshold_value = 30;
  cv::bitwise_and(image, image, text_image_, text_mask_);
  cv::cvtColor(text_image_, text_image_gray_, cv::COLOR_BGR2GRAY);
  cv::threshold(text_image_gray_, text_image_binary_, threshold_value, 255,
                cv::THRESH_BINARY);

  cv::bitwise_and(image, image, background_image_, background_mask_);
  cv::cvtColor(background_image_, background_image_gray_, cv::COLOR_BGR2GRAY);
  cv::threshold(background_image_gray_, background_image_binary_,
                threshold_value, 255, cv::THRESH_BINARY);
}

void Extractor::find_contours() {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(aim_image_binary_, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  for (auto contour : contours) {
    cv::Rect rect = cv::boundingRect(contour);
    if (rect.width > 2.2 * rect.height && rect.width < 4 * rect.height &&
        rect.width > 50) {
      cv::Mat white_img = text_image_binary_(rect);
      int white_cnt = cv::countNonZero(white_img);
      cv::Mat blue_img = background_image_binary_(rect);
      int blue_cnt = cv::countNonZero(blue_img);
      if (white_cnt < 0.3 * rect.width * rect.height &&
          blue_cnt > 0.3 * rect.width * rect.height &&
          white_cnt + blue_cnt > 150 && is_license_plate(origin_image_(rect))) {
        result_bgr_list_.push_back(origin_image_(rect));
      }
    }
  }
}

bool Extractor::is_license_plate(const cv::Mat &image) {
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  std::vector<int> reasonable_list;
  for (int i = 60; i < 180; i++) {
    reasonable_list.clear();
    cv::Mat image_gray_bin;
    if (text_color_ == "white")
      cv::threshold(image_gray, image_gray_bin, i, 255, cv::THRESH_BINARY);
    else if (text_color_ == "black")
      cv::threshold(image_gray, image_gray_bin, i, 255, cv::THRESH_BINARY_INV);
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    int num_labels = cv::connectedComponentsWithStats(image_gray_bin, labels,
                                                      stats, centroids, 4);
    cv::Mat output = cv::Mat::zeros(image.size(), CV_8UC3);
    for (int j = 1; j < num_labels; j++) {
      cv::Mat mask = labels == j;
      output.setTo(cv::Scalar(rand() % 255, rand() % 255, rand() % 255), mask);
    }
    int cnt = 0;
    for (int j = 1; j < num_labels; j++) {
      cv::Mat mask = labels == j;
      cnt += cv::countNonZero(mask);
    }
    for (int j = 1; j < num_labels; j++) {
      cv::Mat mask = labels == j;
      if (1.0 * cv::countNonZero(mask) / cnt < 0.2 &&
          1.0 * cv::countNonZero(mask) / cnt > 0.035) {
        reasonable_list.push_back(j);
      }
    }
    if (reasonable_list.size() >= 7 &&
        num_labels - reasonable_list.size() < 20) {
      break;
    }
  }
  if (reasonable_list.size() < 7) {
    return false;
  }
  return true;
}

void Extractor::get_char_image_list() {
  if (result_bgr_list_.size() == 0) {
    return;
  }
  cv::Mat image = result_bgr_list_[0];
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  std::vector<int> reasonable_list;
  cv::Mat labels;
  int threshold_value = 0;
  for (int i = 60; i < 180; i++) {
    reasonable_list.clear();
    threshold_value = i;
    cv::Mat image_gray_bin;
    if (text_color_ == "white")
      cv::threshold(image_gray, image_gray_bin, i, 255, cv::THRESH_BINARY);
    else if (text_color_ == "black")
      cv::threshold(image_gray, image_gray_bin, i, 255, cv::THRESH_BINARY_INV);
    cv::Mat stats;
    cv::Mat centroids;
    int num_labels = cv::connectedComponentsWithStats(image_gray_bin, labels,
                                                      stats, centroids, 4);
    cv::Mat output = cv::Mat::zeros(image.size(), CV_8UC3);
    for (int j = 1; j < num_labels; j++) {
      cv::Mat mask = labels == j;
      output.setTo(cv::Scalar(rand() % 255, rand() % 255, rand() % 255), mask);
    }
    int cnt = 0;
    for (int j = 1; j < num_labels; j++) {
      cv::Mat mask = labels == j;
      cnt += cv::countNonZero(mask);
    }
    for (int j = 1; j < num_labels; j++) {
      cv::Mat mask = labels == j;
      if (1.0 * cv::countNonZero(mask) / cnt < 0.2 &&
          1.0 * cv::countNonZero(mask) / cnt > 0.035) {
        reasonable_list.push_back(j);
      }
    }
    if (reasonable_list.size() >= 7 &&
        num_labels - reasonable_list.size() < 15) {
      break;
    }
  }
  std::vector<cv::Rect> char_contours;
  double avg_h = 0.0;
  for (auto i : reasonable_list) {
    cv::Mat output = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::Mat mask = labels == i;
    output.setTo(cv::Scalar(255, 255, 255), mask);
    std::vector<std::vector<cv::Point>> sub_char_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(output, sub_char_contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    for (auto j : sub_char_contours) {
      cv::Rect rect = cv::boundingRect(j);
      char_contours.push_back(rect);
      avg_h += rect.height;
    }
  }
  avg_h = 1.0 * avg_h / reasonable_list.size();
  std::sort(char_contours.begin(), char_contours.end(),
            [](cv::Rect a, cv::Rect b) { return a.x < b.x; });
  // double sum_result = cv::mean(image_gray).val[0];
  cv::Mat entire_image_gray_bin;
  // cv::threshold(image_gray, entire_image_gray_bin,
  //               sum_result + 0.16 * image.cols, 255, cv::THRESH_BINARY);
  if (text_color_ == "white")
    cv::threshold(image_gray, entire_image_gray_bin, threshold_value, 255,
                  cv::THRESH_BINARY);
  else if (text_color_ == "black")
    cv::threshold(image_gray, entire_image_gray_bin, threshold_value, 255,
                  cv::THRESH_BINARY_INV);
  debug_image_ = entire_image_gray_bin;
  int width = 0;
  int height = 0;

  for (auto i : char_contours) {
    if (i.height < avg_h * 0.6 || i.height > avg_h * 1.4) {
      continue;
    }
    if (i.width > image.cols / 7) {
      continue;
    }
    width = std::max(width, i.width);
    height = std::max(height, i.height);
  }
  // std::vector<cv::Mat> licence_plate_sig_bin_;
  std::vector<cv::Rect> licence_plate_sig_xywh;
  for (auto i : char_contours) {
    if (i.height < avg_h * 0.6 || i.height > avg_h * 1.4) {
      continue;
    }
    if (i.width > image.cols / 7) {
      continue;
    }
    int centre_x = i.x + i.width / 2;
    int centre_y = i.y + i.height / 2;
    cv::Mat licence_plate_sig_bin_tmp;
    cv::bitwise_not(
        entire_image_gray_bin(cv::Rect(centre_x - width / 2,
                                       centre_y - height / 2, width, height)),
        licence_plate_sig_bin_tmp);
    licence_plate_sig_bin_.push_back(licence_plate_sig_bin_tmp);
    licence_plate_sig_xywh.push_back(i);
  }
  double avg_w = 0;
  avg_h = 0.0;
  double avg_delta_x = 0.0;
  for (int i = 1; i < int(licence_plate_sig_xywh.size()); i++) {
    avg_w += licence_plate_sig_xywh[i].width;
    avg_h += licence_plate_sig_xywh[i].height;
  }
  for (int i = 2; i < int(licence_plate_sig_xywh.size()) - 1; i++) {
    avg_delta_x += licence_plate_sig_xywh[i + 1].x -
                   licence_plate_sig_xywh[i].x -
                   licence_plate_sig_xywh[i].width;
  }
  avg_w = avg_w / (licence_plate_sig_xywh.size() - 1) * 1.1;
  avg_h = avg_h / (licence_plate_sig_xywh.size() - 1) * 1.1;
  avg_delta_x = avg_delta_x / (licence_plate_sig_xywh.size() - 2);
  double x = licence_plate_sig_xywh[1].x - avg_delta_x - avg_w;
  double y = licence_plate_sig_xywh[1].y;
  x = std::max(0.0, x);
  y = std::max(0.0, y);
  avg_w += int(x) + int(avg_w) == int(x + avg_w) ? 0 : 1;
  if (1.0 * abs(licence_plate_sig_xywh[0].height - avg_h) / avg_h > 0.2 ||
      1.0 * abs(licence_plate_sig_xywh[0].width - avg_w) / avg_w > 0.3) {
    cv::bitwise_not(
        entire_image_gray_bin(cv::Rect(int(x), int(y), int(avg_w), int(avg_h))),
        licence_plate_sig_bin_[0]);
  }
}

void Extractor::reset() {
  aim_mask_ = cv::Mat();
  aim_image_ = cv::Mat();
  aim_image_gray_ = cv::Mat();
  aim_image_binary_ = cv::Mat();
  result_bgr_list_.clear();
  licence_plate_sig_bin_.clear();
}
}  // namespace license_detector