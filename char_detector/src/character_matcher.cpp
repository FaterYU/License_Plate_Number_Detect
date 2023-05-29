#include <char_detector/character_matcher.hpp>

namespace license_detector {
CharacterMatcher::CharacterMatcher() { font_read(); }
void CharacterMatcher::font_read() {
  std::string path =
      "/home/fate/project/license_detetor/src/license_detector/char_detector/"
      "font";
  font_ch_name_ = {"澳", "京", "渝", "沪", "津", "冀", "晋", "蒙",
                   "辽", "吉", "黑", "苏", "浙", "皖", "闽", "赣",
                   "鲁", "豫", "鄂", "湘", "粤", "桂", "琼", "川",
                   "贵", "云", "藏", "陕", "甘", "青", "宁", "新"};
  font_word_name_ = {"A", "B", "C", "D", "E", "F", "G", "H",
                     "J", "K", "L", "M", "N", "P", "Q", "R",
                     "S", "T", "U", "V", "W", "X", "Y", "Z"};
  font_num_name_ = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
  for (auto i : font_ch_name_) {
    cv::Mat img = cv::imread(path + "/chinese/140_" + i + ".jpg", 0);
    if (img.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "font_ch_ is empty");
    }
    font_ch_.push_back(img);
  }
  for (auto i : font_word_name_) {
    cv::Mat img = cv::imread(path + "/word/140_" + i + ".jpg", 0);
    if (img.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "font_word_ is empty");
    }
    font_word_.push_back(img);
    font_num_.push_back(img);
  }
  for (auto i : font_num_name_) {
    cv::Mat img = cv::imread(path + "/number/140_" + i + ".jpg", 0);
    if (img.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "font_num_ is empty");
    }
    font_num_.push_back(img);
  }
  font_num_name_ = {"A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M",
                    "N", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
                    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
}

cv::Mat CharacterMatcher::del_white(cv::Mat img) {
  for (int x = 0; x < img.rows; x++) {
    int col = 0;
    for (int y = 0; y < img.cols; y++) {
      if (img.at<uchar>(x, y) == 0) {
        col += 1;
      }
    }
    if (col != 0) {
      img = img(cv::Rect(0, x, img.cols, img.rows - x));
      break;
    }
  }
  for (int x = img.rows - 1; x >= 0; x--) {
    int col = 0;
    for (int y = 0; y < img.cols; y++) {
      if (img.at<uchar>(x, y) == 0) {
        col += 1;
      }
    }
    if (col != 0) {
      img = img(cv::Rect(0, 0, img.cols, x + 1));
      break;
    }
  }
  for (int y = 0; y < img.cols; y++) {
    int row = 0;
    for (int x = 0; x < img.rows; x++) {
      if (img.at<uchar>(x, y) == 0) {
        row += 1;
      }
    }
    if (row != 0) {
      y = std::min(y, 2);
      img = img(cv::Rect(y, 0, img.cols - y, img.rows));
      break;
    }
  }
  for (int y = img.cols - 1; y >= 0; y--) {
    int row = 0;
    for (int x = 0; x < img.rows; x++) {
      if (img.at<uchar>(x, y) == 0) {
        row += 1;
      }
    }
    if (row != 0) {
      y = std::max(y + 1, img.cols - 2);
      img = img(cv::Rect(0, 0, y, img.rows));
      break;
    }
  }
  return img;
}

std::string CharacterMatcher::match_character(
    const license_detector_interfaces::msg::LicenseChar::SharedPtr msg) {
  license_plate_sig_bin.clear();
  for (auto i : msg->license_image) {
    cv::Mat img = cv_bridge::toCvCopy(i, "mono8")->image;
    license_plate_sig_bin.push_back(img);
  }
  std::vector<std::string> result;
  std::vector<int> hamming_distance;
  cv::Mat i = license_plate_sig_bin[0];
  i = del_white(i);
  debug_img = i;
  for (auto j : font_ch_) {
    int width = i.cols;
    int height = i.rows;
    cv::Mat rs_img;
    cv::resize(j, rs_img, cv::Size(width, height));
    hamming_distance.push_back(cv::norm(i, rs_img, cv::NORM_HAMMING));
  }
  result.push_back(font_ch_name_[std::min_element(hamming_distance.begin(),
                                                  hamming_distance.end()) -
                                 hamming_distance.begin()]);
  i = license_plate_sig_bin[1];
  i = del_white(i);
  hamming_distance.clear();
  for (auto j : font_word_) {
    int width = i.cols;
    int height = i.rows;
    cv::Mat rs_img;
    cv::resize(j, rs_img, cv::Size(width, height));
    cv::Mat rs_img_bin;
    cv::threshold(rs_img, rs_img_bin, 100, 255, cv::THRESH_BINARY);
    hamming_distance.push_back(cv::norm(i, rs_img_bin, cv::NORM_HAMMING));
  }
  result.push_back(font_word_name_[std::min_element(hamming_distance.begin(),
                                                    hamming_distance.end()) -
                                   hamming_distance.begin()]);
  license_plate_sig_bin.erase(license_plate_sig_bin.begin());
  license_plate_sig_bin.erase(license_plate_sig_bin.begin());
  for (auto i : license_plate_sig_bin) {
    cv::Mat img = del_white(i);
    hamming_distance.clear();
    for (auto j : font_num_) {
      int width = img.cols;
      int height = img.rows;
      cv::Mat rs_img;
      cv::resize(j, rs_img, cv::Size(width, height));
      hamming_distance.push_back(cv::norm(img, rs_img, cv::NORM_HAMMING));
    }
    result.push_back(font_num_name_[std::min_element(hamming_distance.begin(),
                                                     hamming_distance.end()) -
                                    hamming_distance.begin()]);
  }
  std::string res;
  for (auto i : result) {
    res += i;
  }
  return res;
}

}  // namespace license_detector