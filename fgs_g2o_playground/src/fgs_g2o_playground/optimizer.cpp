#include "fgs_g2o_playground/optimizer.hpp"

#include <memory>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace fgs {
namespace g2o_playground {

Optimizer::Ptr Optimizer::Create(const std::string& optimization_algo_type) {
  using MyBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>>;
  using MyLinearSolver = g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType>;

  g2o::OptimizationAlgorithmWithHessian* opt_algo;
  if (optimization_algo_type == "levenberg") {
    opt_algo = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<MyBlockSolver>(std::make_unique<MyLinearSolver>()));
  } else if (optimization_algo_type == "gauss_newton") {
    opt_algo = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<MyBlockSolver>(std::make_unique<MyLinearSolver>()));
  } else if (optimization_algo_type == "dogleg") {
    opt_algo = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<MyBlockSolver>(std::make_unique<MyLinearSolver>()));
  }

  Ptr optimizer(new g2o::SparseOptimizer());
  optimizer->setAlgorithm(opt_algo);

  return optimizer;
}

}
}
