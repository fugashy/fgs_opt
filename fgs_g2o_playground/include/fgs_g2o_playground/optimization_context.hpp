#ifndef FGS_G2O_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#define FGS_G2O_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_

#include "g2o/core/block_solver.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include <opencv2/core.hpp>

#include "fgs_g2o_playground/curve.hpp"

namespace fgs {
namespace g2o_playground {

void CurveFitting(const cv::Mat& cv_data_array) {
  std::vector<CurveData*> data_array;
  data_array.reserve(cv_data_array.rows);
  for (int i = 0; i < cv_data_array.rows; ++i) {
    CurveData* data;
    data->SetData(cv_data_array.row(i));
    data_array[i] = data;
  }

  CurveParameter* param(new CurveParameter());
  param->Init(100.0);

  // Cereate optimizer
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>>
      MyBlockSolver;
  typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType>
      MyLinearSolver;
  g2o::SparseOptimizer optimizer;
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
  optimizer.setAlgorithm(solver);

  param->setId(0);
  optimizer.addVertex(param);

  for (int i = 0; i < cv_data_array.rows; ++i) {
    data_array[i]->setVertex(0, param);
    optimizer.addEdge(data_array[i]);
  }

  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(10);

  param->ShowParam();
}
}
}
#endif
