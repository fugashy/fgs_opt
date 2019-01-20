#ifndef FGS_G2O_PLAYGROUND_OPTIMIZER_HPP_
#define FGS_G2O_PLAYGROUND_OPTIMIZER_HPP_

#include <memory>

#include <g2o/core/sparse_optimizer.h>

namespace fgs {
namespace g2o_playground {

class Optimizer {
 public:
  typedef std::unique_ptr<g2o::SparseOptimizer> Ptr;
  static Ptr Create(const std::string& optimization_algo_type);
};

}
}

#endif
