#include "fgs_ceres_playground/optimization_context.hpp"
#include "fgs_ceres_playground/line_fitting.hpp"

#include <fgs_opt_data_storage/loader.hpp>

using fgs::ceres_playground::Point2d;
using fgs::ceres_playground::Line2dResidual;
using fgs::ceres_playground::ByAutoDiffOptimizationContext;
using fgs::opt_data_storage::LoadCVYaml;


int main(int argc, char** argv) {
  cv::Mat data = LoadCVYaml(argv[1]);

  ByAutoDiffOptimizationContext<Line2dResidual> solver(data);

  ceres::Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;

  solver.Solve(options, summary);

  return 0;
}
