#ifndef FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT__FACTORY_HPP_
#define FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT__FACTORY_HPP_
#include <string>

#include "fgs_ceres_playground/optimization_context.hpp"

namespace fgs {
namespace ceres_playground {

class OptimizationContextFactory {
 public:
  static OptimizationContext::Ptr Create(const std::string& cv_storage_path, const std::string& type);
};

}
}
#endif
