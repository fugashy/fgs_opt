#include "fgs_ceres_playground/bundle_adjustment_in_the_large.hpp"

#include <opencv2/core.hpp>

namespace fgs {
namespace ceres_playground {

BundleAdjustmentInTheLarge::BundleAdjustmentInTheLarge(const std::string& cv_storage_path) {
  cv::FileStorage fs(cv_storage_path, cv::FileStorage::READ);

  // 観測データ
  fs["observations"] >> observations_;

  // 最適化パラメータ初期値
  fs["camera_parameters"] >> camera_parameters_;
  fs["points"] >> points_;
}

}
}

