#include "fgs_g2o_playground/optimizer.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace fgs {
namespace g2o_playground {

Optimizer::Ptr Optimizer::Create(const std::string& optimization_algo_type) {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>>
      MyBlockSolver;
  typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType>
      MyLinearSolver;

  g2o::OptimizationAlgorithmWithHessian* opt_algo;
  if (optimization_algo_type == "levenberg") {
    opt_algo = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
  } else if (optimization_algo_type == "gauss_newton") {
    opt_algo = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
  } else if (optimization_algo_type == "dogleg") {
    opt_algo = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
  }

  Ptr optimizer(new g2o::SparseOptimizer());
  optimizer->setAlgorithm(opt_algo);

  return optimizer;
}

}
}
