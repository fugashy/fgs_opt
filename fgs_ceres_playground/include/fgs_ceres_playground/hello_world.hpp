#ifndef FGS_CERES_PLAYGROUND_HELLO_WORLD_HPP_
#define FGS_CERES_PLAYGROUND_HELLO_WORLD_HPP_
#include <ceres/ceres.h>

#include "fgs_ceres_playground/data.hpp"
#include "fgs_ceres_playground/parameter.hpp"

namespace fgs {
namespace ceres_playground {

// y = 0.5(10 - x)^2
struct AutomaticDiffHelloWorld {
  template<typename T>
  bool operator()(const T* const param, T* residual) const {
    residual[0] = T(10.0) - param[0];
    return true;
  }
};

struct NumericDiffHelloWorld {
  bool operator()(const double* const param, double* residual) const {
    residual[0] = 10.0 - param[0];
    return true;
  }
};

class AnalyticDiffHelloWorld : public ceres::SizedCostFunction<1, 1> {
 public:
  virtual ~AnalyticDiffHelloWorld() {}

  virtual bool Evaluate(double const* const* param,     /* (in) mat */
                        double      *        residuals, /* (out)vec */
                        double      *      * jacobians  /* (out)mat */) const {
    residuals[0] = 10.0 - param[0][0];

    if (jacobians != NULL && jacobians[0] != NULL) {
      jacobians[0][0] = -1.0;
    }

    return true;
  }
};
}
}
#endif
