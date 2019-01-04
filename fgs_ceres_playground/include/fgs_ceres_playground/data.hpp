#ifndef FGS_CERES_PLAYGROUND_DATA_HPP_
#define FGS_CERES_PLAYGROUND_DATA_HPP_
#include <vector>

#include <opencv2/core.hpp>

namespace fgs {
namespace ceres_playground {

struct Value : private std::vector<double> {
  Value() {
    this->resize(1);
    (*this)[0] = 0.0;
  }
  Value(double value) {
    this->resize(1);
    (*this)[0] = value;
  }
  ~Value() {
    std::vector<double>().swap(*this);
  }

  static void CvToDataArray(
      const cv::Mat& data_mat, std::vector<Value>& data_array) {
    if (data_mat.empty()) {
      throw std::runtime_error("data mat is empty");
    } else if (data_mat.rows == 0 || data_mat.cols != 1) {
      throw std::runtime_error("data mat size is invalid");
    }
    std::vector<Value>().swap(data_array);
    data_array.resize(data_mat.rows);
    for (int i = 0; i < data_mat.rows; ++i) {
      Value data(data_mat.at<double>(i, 0));
      data_array[i] = data;
    }
  }

  using std::vector<double>::operator[];
};

struct Point2d : private std::vector<double> {
  Point2d() {
    this->resize(2);
    (*this)[0] = (*this)[1] = 0.0;
  }
  Point2d(double x, double y) {
    this->resize(2);
    (*this)[0] = x;
    (*this)[1] = y;
  }
  ~Point2d() {
    std::vector<double>().swap(*this);
  }

  void x(double x) { (*this)[0] = x; }
  void y(double y) { (*this)[1] = y; }

  double get_x() const { return (*this)[0]; }
  double get_y() const { return (*this)[1]; }

  static void CvToDataArray(
      const cv::Mat& data_mat, std::vector<Point2d>& data_array) {
    if (data_mat.empty()) {
      throw std::runtime_error("data mat is empty");
    } else if (data_mat.rows == 0 || data_mat.cols != 2) {
      throw std::runtime_error("data mat size is invalid");
    }
    std::vector<Point2d>().swap(data_array);
    data_array.resize(data_mat.rows);
    for (int i = 0; i < data_mat.rows; ++i) {
      Point2d data(data_mat.at<double>(i, 0), data_mat.at<double>(i, 1));
      data_array[i] = data;
    }
  }

  using std::vector<double>::operator[];
};

}
}
#endif
