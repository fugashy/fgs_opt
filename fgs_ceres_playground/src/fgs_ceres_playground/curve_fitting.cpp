#include "fgs_ceres_playground/curve_fitting.hpp"
#include <fgs_opt_data_storage/loader.hpp>

#include <ceres/ceres.h>

using fgs::ceres_playground::Point2d;
using fgs::ceres_playground::SimpleCurve2dResidual;
using fgs::opt_data_storage::LoadCVYaml;

int main(int argc, char** argv) {
  double a = 100;

  cv::Mat data = LoadCVYaml(argv[1]);

  ceres::Problem problem;
  for (int i = 0; i < data.rows; ++i) {
    Point2d<double> p;
    p.x = data.at<double>(i, 0);
    p.y = data.at<double>(i, 1);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<SimpleCurve2dResidual, 1, 1>(
            new SimpleCurve2dResidual(p)),
        NULL,
        &a);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;
  std::cout << a << std::endl;
  return EXIT_SUCCESS;
}
