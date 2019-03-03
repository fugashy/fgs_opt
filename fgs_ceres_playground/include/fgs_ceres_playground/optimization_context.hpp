#ifndef FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#define FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#include <memory>

#include <ceres/ceres.h>
#include <opencv2/core.hpp>

#include "fgs_ceres_playground/data.hpp"
#include "fgs_ceres_playground/bundle_adjustment_in_the_large.hpp"

namespace fgs {
namespace ceres_playground {

template<class ResidualType>
class FittingContext {
 public:
  FittingContext(const cv::Mat& data_mat) {
    param_.Init(100.0);

    typename ResidualType::DataArrayType data_array;
    ResidualType::DataType::CvToDataArray(data_mat, data_array);
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

template<class ResidualType>
class BALContext {
 public:
  BALContext(const std::string& cv_storage_path) :
      problem_source_(new BundleAdjustmentInTheLarge(cv_storage_path)) {
    for (int i = 0; i < problem_source_->camera_num(); ++i) {
      const cv::Mat data = problem_source_->observation_data(i);
      double* camera_parameter = problem_source_->camera_parameter(i);
      double* point = problem_source_->point(i);
      ceres::CostFunction* cf = ResidualType::Create(data);
      problem_.AddResidualBlock(cf, NULL, camera_parameter, point);
    }
  }

  void Solve(ceres::Solver::Options& options) {
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);
  }

 private:
  std::unique_ptr<BundleAdjustmentInTheLarge> problem_source_;
  ceres::Problem problem_;
};
}
}
#endif
