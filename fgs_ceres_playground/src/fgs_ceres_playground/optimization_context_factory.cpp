#include "fgs_ceres_playground/optimization_context_factory.hpp"

#include "fgs_ceres_playground/circle_fitting.hpp"
#include "fgs_ceres_playground/curve_fitting.hpp"
#include "fgs_ceres_playground/line_fitting.hpp"
#include "fgs_ceres_playground/simple_ba.hpp"

namespace fgs {
namespace ceres_playground {

OptimizationContext::Ptr OptimizationContextFactory::Create(
    const std::string& cv_storage_path, const std::string& type) {
  OptimizationContext::Ptr ret_ptr;

  if (type == "simple_ba") {
    ret_ptr.reset(new BALContext<SimpleBAResidual>(cv_storage_path));
  } else if (type == "curve2d") {
    ret_ptr.reset(new FittingContext<SimpleCurve2dResidual>(cv_storage_path));
  } else if (type == "circle2d") {
    ret_ptr.reset(new FittingContext<Circle2dResidual>(cv_storage_path));
  } else if (type == "line2d") {
    ret_ptr.reset(new FittingContext<Line2dResidual>(cv_storage_path));
  } else {
    throw std::runtime_error("Invalid type");
  }

  return ret_ptr;
}

}
}
