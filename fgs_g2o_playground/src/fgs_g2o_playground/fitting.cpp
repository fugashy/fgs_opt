#include <fgs_opt_data_storage/loader.hpp>

#include "fgs_g2o_playground/curve.hpp"
#include "fgs_g2o_playground/line.hpp"
#include "fgs_g2o_playground/optimization_context.hpp"

using fgs::g2o_playground::CurveFitting;
using fgs::g2o_playground::LineFitting;
using fgs::opt_data_storage::LoadCVYaml;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "usage: rosrun fgs_g2o_playground "
                 "fgs_g2o_playground_fitting data_path curve" << std::endl;
    return -1;
  }
  cv::Mat data = LoadCVYaml(argv[1]);
  const std::string type = argv[2];

  if (type == "curve") {
    CurveFitting(data);
  } else if (type == "line") {
    LineFitting(data);
  } else {
    std::cerr << type << " is not implemented" << std::endl;
    return -1;
  }

  return 0;
}
