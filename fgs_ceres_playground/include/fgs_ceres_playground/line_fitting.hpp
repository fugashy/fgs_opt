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

  explicit Line2dResidual(const DataType& data) : data_(data) {}

  template<typename T>
  bool operator() (const T* parameter, T* residual) const {
    residual[0] = data_[1] - (parameter[0] * data_[0] + parameter[1]);

    return true;
  }

  static void ShowParam(const ParameterType& param) {
    std::cout << '\t' << "a: " << param[0] << std::endl
              << '\t' << "b: " << param[1] << std::endl;
  }

  const static int DimResidual;
  const static int DimParam;

 private:
  DataType data_;
};

const int Line2dResidual::DimResidual = 1;
const int Line2dResidual::DimParam = 2;

}
}
#endif