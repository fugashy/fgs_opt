#ifndef FGS_CERES_PLAYGROUND_CURVE_FITTING_HPP_
#define FGS_CERES_PLAYGROUND_CURVE_FITTING_HPP_
#include "fgs_ceres_playground/data.hpp"

namespace fgs {
namespace ceres_playground {

struct SimpleCurve2dResidual {
  explicit SimpleCurve2dResidual(const Point2d<double>& p) : p_(p) {}

  template<typename T>
  bool operator() (const T* parameter, T* residual) const {
    residual[0] = p_.y - parameter[0] * p_.x * p_.x;

    return true;
  }

 private:
  Point2d<double> p_;
};

}
}
#endif
