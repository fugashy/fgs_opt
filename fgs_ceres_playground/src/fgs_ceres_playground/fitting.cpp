#include <memory>
#include "fgs_ceres_playground/optimization_context.hpp"
#include "fgs_ceres_playground/line_fitting.hpp"
#include "fgs_ceres_playground/curve_fitting.hpp"

#include <fgs_opt_data_storage/loader.hpp>

using fgs::ceres_playground::Point2d;
using fgs::ceres_playground::Line2dResidual;
using fgs::ceres_playground::SimpleCurve2dResidual;
using fgs::ceres_playground::ByAutoDiffOptimizationContext;
using fgs::opt_data_storage::LoadCVYaml;


int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "usage: rosrun fgs_ceres_playground "
                 "fgs_ceres_playground_fitting data_path line2d" << std::endl;
    return -1;
  }
  cv::Mat data = LoadCVYaml(argv[1]);

  ceres::Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  const std::string type(argv[2]);
  if (type == "line2d") {
    ByAutoDiffOptimizationContext<Line2dResidual> solver(data);
    solver.Solve(options);
  } else if (type == "curve2d") {
    ByAutoDiffOptimizationContext<SimpleCurve2dResidual> solver(data);
    solver.Solve(options);
  }

  return 0;
}
