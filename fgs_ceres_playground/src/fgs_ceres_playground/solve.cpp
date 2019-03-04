#include "fgs_ceres_playground/optimization_context_factory.hpp"

using fgs::ceres_playground::OptimizationContext;
using fgs::ceres_playground::OptimizationContextFactory;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "usage: rosrun fgs_ceres_playground "
                 "fgs_ceres_playground_solve data_path line2d" << std::endl;
    return -1;
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 4;

  OptimizationContext::Ptr solver = OptimizationContextFactory::Create(argv[1], argv[2]);
  solver->Solve(options);

  return 0;
}
