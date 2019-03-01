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
class ComplexProblemOptimizationContext {
 public:
  ComplexProblemOptimizationContext(const std::vector<cv::Mat>& data_mat) {
    param_.Init(100.0);

    typename ResidualType::DataTypeArray data_array;
    typename ResidualType::ExtendDataTypeArray extend_data_array;
    ResidualType::DataType::CvToDataArray(data_mat[0], data_array);
    ResidualType::ExtendDataType::CvToDataArray(data_mat[1], extend_data_array);
    std::cout << "size of data       : " << data_array.size() << std::endl
              << "size of extend data: " << extend_data_array.size() << std::endl;
    auto it_data = data_array.begin();
    auto it_extend_data = extend_data_array.begin();
    for (; it_data != data_array.end(); ++it_data, ++it_extend_data) {
      ceres::CostFunction* cf = ResidualType::Create(*it_data);
      problem_.AddResidualBlock(cf, NULL, &param_[0], &((*it_extend_data)[0]));
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
