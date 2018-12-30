#ifndef FGS_CERES_PLAYGROUND_CIRCLE_FITTING_HPP_
#define FGS_CERES_PLAYGROUND_CIRCLE_FITTING_HPP_
#include "fgs_ceres_playground/data.hpp"
#include "fgs_ceres_playground/parameter.hpp"

namespace fgs {
namespace ceres_playground {

struct Circle2dResidual {
  typedef Point2d DataType;
  typedef std::vector<DataType> DataArrayType;

  typedef Parameter<3> ParameterType;

  explicit Circle2dResidual(const DataType& data) : data_(data) {}

  template<typename T>
  bool operator() (const T* parameter, T* residual) const {
    const auto r2 = parameter[0] * parameter[0];
    const auto dx2 = (data_[0] - parameter[1]) * (data_[0] - parameter[1]);
    const auto dy2 = (data_[1] - parameter[2]) * (data_[1] - parameter[2]);

    // which is the distance of the sample from the circle. This works
    // reasonably well, but the sqrt() adds strong nonlinearities to the cost
    // function. Instead, a different cost is used, which while not strictly a
    // distance in the metric sense (it has units distance^2) it produces more
    // robust fits when there are outliers. This is because the cost surface is
    // more convex.
    residual[0] = r2 - (dx2 + dy2);

    return true;
  }

  static void ShowParam(const ParameterType& param) {
    std::cout << '\t' << "r: " << param[0] << std::endl
              << '\t' << "tx: " << param[1] << std::endl
              << '\t' << "ty: " << param[2] << std::endl;
  }

  static ceres::CostFunction* Create(const DataType& data) {
    return (new ceres::AutoDiffCostFunction<Circle2dResidual, 1, 3>(
          new Circle2dResidual(data)));
  }

 private:
  DataType data_;
};

}
}

#endif
