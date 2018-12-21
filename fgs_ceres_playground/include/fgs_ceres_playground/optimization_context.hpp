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
  ByAutoDiffOptimizationContext(const cv::Mat& data_mat) {
    param_.Init();

    std::vector<typename ResidualType::DataType> data_array;
    ResidualType::CvToDataArray(data_mat, data_array);
    for (auto it = data_array.begin(); it != data_array.end(); ++it) {
      problem_.AddResidualBlock(
          new ceres::AutoDiffCostFunction<
              ResidualType,
              ResidualType::D,
              ResidualType::Parameter::D>(new ResidualType((*it))), NULL, &param_[0]);
    }
  }

  void Solve(ceres::Solver::Options& options) {
    std::cout << "Before" << std::endl;
    param_.Show();

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);

    std::cout << "After" << std::endl;
    param_.Show();
  }

 private:
  typename ResidualType::Parameter param_;
  ceres::Problem problem_;
  cv::Mat data_;
};
}
}
#endif
