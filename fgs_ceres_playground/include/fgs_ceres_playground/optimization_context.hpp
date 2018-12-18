#ifndef FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#define FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#include <ceres/ceres.h>
#include <opencv2/core.hpp>

#include "fgs_ceres_playground/data.hpp"

namespace fgs {
namespace ceres_playground {

template<class T>
class ByAutoDiffOptimizationContext {
 public:
  ByAutoDiffOptimizationContext(const cv::Mat& data) {
    param_.Init();

    for (int i = 0; i < data.rows; ++i) {
      Point2d<double> p;
      p.x = data.at<double>(i, 0);
      p.y = data.at<double>(i, 1);
      problem_.AddResidualBlock(
          new ceres::AutoDiffCostFunction<T, 1, 2>(new T(p)), NULL, &param_[0]);
    }
  }

  void Solve(ceres::Solver::Options& options, ceres::Solver::Summary& summary) {
    summary = ceres::Solver::Summary();

    ceres::Solve(options, &problem_, &summary);
    std::cout << summary.BriefReport() << std::endl;

    for (auto it = param_.begin(); it != param_.end(); ++it) {
      std::cout << "result: " << (*it) << std::endl;
    }
  }

 private:
  typename T::Parameter param_;
  ceres::Problem problem_;
  cv::Mat data_;
};
}
}
#endif
