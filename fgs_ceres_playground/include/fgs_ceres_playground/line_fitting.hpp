#ifndef FGS_CERES_PLAYGROUND_LINE_FITTING_HPP_
#define FGS_CERES_PLAYGROUND_LINE_FITTING_HPP_
#include "fgs_ceres_playground/data.hpp"

namespace fgs {
namespace ceres_playground {

struct Line2dResidual {
  struct Parameter : public std::vector<double> {
    Parameter() {
      this->resize(2);
      (*this)[0] = (*this)[1] = 0.0;
    }
    void Init() {
      (*this)[0] = 3.0;
      (*this)[1] = 6.0;
    }
    void Show() {
      std::cout << "a: " << (*this)[0] << std::endl
                << "b: " << (*this)[1] << std::endl;
    }
    const static int D;

  };

  typedef Point2d DataType;
  typedef std::vector<DataType> DataArrayType;


  explicit Line2dResidual(const DataType& data) : data_(data) {}

  template<typename T>
  bool operator() (const T* parameter, T* residual) const {
    residual[0] = data_[1] - (parameter[0] * data_[0] + parameter[1]);

    return true;
  }

  const static int D;

  static void CvToDataArray(
      const cv::Mat& data_mat, DataArrayType& data_array) {
    if (data_mat.empty()) {
      throw std::runtime_error("data mat is empty");
    } else if (data_mat.rows == 0 || data_mat.cols != 2) {
      throw std::runtime_error("data mat size is invalid");
    }
    DataArrayType().swap(data_array);
    data_array.resize(data_mat.rows);
    for (int i = 0; i < data_mat.rows; ++i) {
      DataType data(data_mat.at<double>(i, 0), data_mat.at<double>(i, 1));
      data_array[i] = data;
    }
  }

 private:
  DataType data_;
};

const int Line2dResidual::D = 1;
const int Line2dResidual::Parameter::D = 2;

}
}
#endif
