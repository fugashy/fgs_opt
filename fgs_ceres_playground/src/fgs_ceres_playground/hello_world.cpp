#include <memory>
#include <ceres/ceres.h>

#include "fgs_ceres_playground/hello_world.hpp"
#include "fgs_ceres_playground/parameter.hpp"

using fgs::ceres_playground::Parameter;
using fgs::ceres_playground::AutomaticDiffHelloWorld;
using fgs::ceres_playground::NumericDiffHelloWorld;
using fgs::ceres_playground::AnalyticDiffHelloWorld;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: rosrun fgs_ceres_playground "
                 "fgs_ceres_playground_hello_world auto" << std::endl;
    return -1;
  }
  const std::string diff_mode(argv[1]);
  fgs::ceres_playground::Parameter<1> param;
  param.Init(10000.0);

  std::cout << "Before" << std::endl
            << '\t' << "x: " << param[0] << std::endl;

  ceres::Problem problem;

  ceres::CostFunction* cost_function;
  if (diff_mode == "auto") {
    std::cout << "use automatic diff cost function class" << std::endl;
    cost_function =
        new ceres::AutoDiffCostFunction<AutomaticDiffHelloWorld, 1, 1>(
            new AutomaticDiffHelloWorld());
  } else if (diff_mode == "numeric") {
    std::cout << "use numeric diff cost function class" << std::endl;
    cost_function =
        new ceres::NumericDiffCostFunction<NumericDiffHelloWorld, ceres::CENTRAL, 1, 1>(
            new NumericDiffHelloWorld());
  } else if (diff_mode == "analytic") {
    std::cout << "use analytic diff cost function class" << std::endl;
    cost_function = new AnalyticDiffHelloWorld();
  } else {
    std::cerr << "use default diff: auto" << std::endl;
    cost_function =
        new ceres::AutoDiffCostFunction<AutomaticDiffHelloWorld, 1, 1>(
            new AutomaticDiffHelloWorld());
  }
  problem.AddResidualBlock(cost_function, NULL, &param[0]);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "After" << std::endl
            << '\t' << "x: " << param[0] << std::endl;
  return 0;
}
