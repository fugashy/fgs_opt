#include "fgs_ceres_playground/bundle_adjustment_in_the_large.hpp"

#include <iostream>

#include <opencv2/core.hpp>

namespace fgs {
namespace ceres_playground {

BundleAdjustmentInTheLarge::BundleAdjustmentInTheLarge(const std::string& cv_storage_path) {
  cv::FileStorage fs(cv_storage_path, cv::FileStorage::READ);

  // 観測データ
  fs["observations"] >> observations_;
  std::cout << "observations: ("
            << observations_.rows << "," << observations_.cols << ")" << std::endl;

  // 最適化パラメータ初期値
  fs["camera_parameters"] >> camera_parameters_;
  std::cout << "camera_parameters: ("
            << camera_parameters_.rows << "," << camera_parameters_.cols << ")" << std::endl;
  fs["points"] >> points_;
  std::cout << "points: ("
            << points_.rows << "," << points_.cols << ")" << std::endl;
}

}
}

