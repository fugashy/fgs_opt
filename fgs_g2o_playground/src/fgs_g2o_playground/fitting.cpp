#include "fgs_g2o_playground/optimization_context.hpp"
#include "fgs_g2o_playground/curve.hpp"

#include <fgs_opt_data_storage/loader.hpp>

using fgs::g2o_playground::CurveFitting;
using fgs::opt_data_storage::LoadCVYaml;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "usage: rosrun fgs_ceres_playground "
                 "fgs_ceres_playground_fitting data_path line2d" << std::endl;
    return -1;
  }
  cv::Mat data = LoadCVYaml(argv[1]);
  const std::string type = argv[2];

  if (type == "curve") {
    CurveFitting(data);
  } else {
    std::cerr << type << " is not implemented" << std::endl;
    return -1;
  }

  return 0;
}
