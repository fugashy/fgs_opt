#ifndef FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#define FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#include <ceres/ceres.h>
#include <opencv2/core.hpp>

#include "fgs_ceres_playground/data.hpp"

namespace fgs {
namespace ceres_playground {

template<class ResidualType>
class ByAutoDiffOptimizationContext {
 public:
  ByAutoDiffOptimizationContext(const std::vector<cv::Mat>& data_mat) {
    param_.Init(100.0);

    typename ResidualType::DataArrayType data_array;
    ResidualType::DataType::CvToDataArray(data_mat[0], data_array);
    for (auto it = data_array.begin(); it != data_array.end(); ++it) {
      problem_.AddResidualBlock(ResidualType::Create(*it), NULL, &param_[0]);
    }
  }

  void Solve(ceres::Solver::Options& options) {
    std::cout << "Before" << std::endl;
    ResidualType::ShowParam(param_);

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);

    std::cout << "After" << std::endl;
    ResidualType::ShowParam(param_);
  }

 private:
  typename ResidualType::ParameterType param_;
  ceres::Problem problem_;
  cv::Mat data_;
};
}
}
#endif
