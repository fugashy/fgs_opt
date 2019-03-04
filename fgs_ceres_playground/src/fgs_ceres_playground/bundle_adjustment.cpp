#include <memory>

#include "fgs_ceres_playground/optimization_context.hpp"
#include "fgs_ceres_playground/simple_ba.hpp"

#include <fgs_opt_data_storage/loader.hpp>

using fgs::ceres_playground::Point2d;
using fgs::ceres_playground::SimpleBAResidual;
using fgs::ceres_playground::BALContext;
using fgs::opt_data_storage::LoadCVYaml;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "usage: rosrun fgs_ceres_playground "
                 "fgs_ceres_playground_bundle_adjustment data_path simple_ba" << std::endl;
    return -1;
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  const std::string type(argv[2]);
  if (type == "simple_ba") {
    BALContext<SimpleBAResidual> solver(argv[1]);
    solver.Solve(options);
  } else {
    std::cerr << type << " is not implemented." << std::endl;
    return -1;
  }

  return 0;
}
