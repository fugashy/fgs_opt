#include <memory>

#include "fgs_g2o_playground/optimization_context.hpp"

using fgs::g2o_playground::Curve;
using fgs::g2o_playground::Line;
using fgs::g2o_playground::OptimizationContext;
using fgs::g2o_playground::FittingContext;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "usage: rosrun fgs_g2o_playground "
                 "fgs_g2o_playground_fitting data_path curve" << std::endl;
    return -1;
  }
  const std::string type = argv[2];

  std::shared_ptr<OptimizationContext> optimizer;
  if (type == "curve") {
    optimizer = std::make_shared<FittingContext<Curve>>(argv[1]);
  } else if (type == "line") {
    optimizer = std::make_shared<FittingContext<Line>>(argv[1]);
  } else {
    std::cerr << type << " is not implemented" << std::endl;
    return -1;
  }

  optimizer->Optimize();

  return 0;
}
