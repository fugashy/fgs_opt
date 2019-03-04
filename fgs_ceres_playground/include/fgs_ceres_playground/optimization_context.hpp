#ifndef FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#define FGS_CERES_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#include <memory>

#include <ceres/ceres.h>

#include "fgs_ceres_playground/cv_viz.hpp"
#include "fgs_ceres_playground/data.hpp"
#include "fgs_ceres_playground/bundle_adjustment_in_the_large.hpp"

namespace fgs {
namespace ceres_playground {

class OptimizationContext {
 public:
  typedef std::shared_ptr<OptimizationContext> Ptr;
  virtual void Solve(ceres::Solver::Options& options) = 0;
};

template<class ResidualType>
class FittingContext : public OptimizationContext {
 public:
  FittingContext(const std::string& cv_storage_path) {
    cv::FileStorage fs(cv_storage_path, cv::FileStorage::READ);
    cv::Mat data;
    fs["data"] >> data;
    if (data.empty()) {
      throw std::runtime_error("data is empty");
    }

    param_.Init(100.0);

    typename ResidualType::DataArrayType data_array;
    ResidualType::DataType::CvToDataArray(data, data_array);
    for (auto it = data_array.begin(); it != data_array.end(); ++it) {
      problem_.AddResidualBlock(ResidualType::Create(*it), NULL, &param_[0]);
    }
  }

  virtual void Solve(ceres::Solver::Options& options) {
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
};

template<class ResidualType>
class BALContext : public OptimizationContext {
 public:
  BALContext(const std::string& cv_storage_path) :
      problem_source_(new BundleAdjustmentInTheLarge(cv_storage_path)) {
    for (int i = 0; i < problem_source_->observations_num(); ++i) {
      const cv::Mat data = problem_source_->observation_data(i);
      double* camera_parameter = problem_source_->param_associated_with_obs(
          i, BundleAdjustmentInTheLarge::Item::Camera);
      double* point = problem_source_->param_associated_with_obs(
          i, BundleAdjustmentInTheLarge::Item::Point);
      ceres::CostFunction* cf = ResidualType::Create(data);
      problem_.AddResidualBlock(cf, NULL, camera_parameter, point);
    }
  }

  virtual void Solve(ceres::Solver::Options& options) {
    CVBALVisualizer viz(problem_source_, "BAL");
    viz.AddNoise(0.0, 0.1);
    viz.Show();

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);

    viz.Show();
  }

 private:
  std::shared_ptr<BundleAdjustmentInTheLarge> problem_source_;
  ceres::Problem problem_;
};
}
}
#endif
