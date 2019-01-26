#ifndef FGS_CERES_PLAYGROUND_LINE_FITTING_HPP_
#define FGS_CERES_PLAYGROUND_LINE_FITTING_HPP_
#include "fgs_ceres_playground/data.hpp"
#include "fgs_ceres_playground/parameter.hpp"

namespace fgs {
namespace ceres_playground {

struct Line2dResidual {
  typedef Point2d DataType;
  typedef std::vector<DataType> DataArrayType;

  typedef Parameter<2> ParameterType;

  explicit Line2dResidual(const DataType& data) : data_(data), error_type_("y-fx") {}
  explicit Line2dResidual(const DataType& data, const std::string& error_type)
    : data_(data), error_type_(error_type) {
    if (error_type_ != "y-fx" && error_type_ != "perpendicular_line") {
      throw std::runtime_error("Invalid error type");
    }
  }

  template<typename T>
  bool operator() (const T* parameter, T* residual) const {
    if (error_type_ == "y-fx") {
      residual[0] = data_[1] - (parameter[0] * data_[0] + parameter[1]);
    } else if (error_type_ == "perpendicular_line"){
      // (yi - (a* xi + b))^2
      // --------------------
      //       1 + a^2
      const T sqrt_denominator = data_[1] - (parameter[0] * data_[0] + parameter[1]);
      const T denominator = sqrt_denominator * sqrt_denominator;
      const T numerator = T(1.0) + (parameter[0] * parameter[0]);
      residual[0] = denominator / numerator;
    }

    return true;
  }

  static void ShowParam(const ParameterType& param) {
    std::cout << '\t' << "a: " << param[0] << std::endl
              << '\t' << "b: " << param[1] << std::endl;
  }

  static ceres::CostFunction* Create(
      const DataType& data, const std::string& error_type = "perpendicular_line") {
    return (new ceres::AutoDiffCostFunction<Line2dResidual, 1, 2>(
          new Line2dResidual(data, error_type)));
  }

 private:
  DataType data_;
  std::string error_type_;
};

}
}
#endif
