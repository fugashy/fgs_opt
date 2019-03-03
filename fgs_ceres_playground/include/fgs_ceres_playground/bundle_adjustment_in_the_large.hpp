#ifndef FGS_CERES_PLAYGROUND_BUNDLE_ADJUSTMENT_IN_THE_LARGE_HPP_
#define FGS_CERES_PLAYGROUND_BUNDLE_ADJUSTMENT_IN_THE_LARGE_HPP_
#include <iostream>
#include <string>

#include <opencv2/core.hpp>

namespace fgs {
namespace ceres_playground {

// Bundle Adjustment in the Large データセットを読み込む
// 残差ブロック定義時に便利になるようなアクセス方法を提供する
class BundleAdjustmentInTheLarge {
 public:
  explicit BundleAdjustmentInTheLarge(const std::string& cv_storage_path);

  // 観測画像点
  // camera_idとpoint_idを飛ばしてアクセスしたいので列番号は2
  double* observation(int i) const {
    return const_cast<double*>(&(observations_.at<double>(i, 2)));
  }
  cv::Mat observation_data(int i) const {
    return (cv::Mat_<double>(1, 2) << (*(observation(i))), (*(observation(i) + 1)));
  }

  // 観測画像点に対応したカメラパラメータへのアドレス
  double* camera_parameter(int obs_index) const {
    const int camera_index = observations_.at<double>(obs_index, 0);
    return const_cast<double*>(&(camera_parameters_.at<double>(camera_index, 0)));
  }
  // 観測画像点に対応した点へのアドレス
  double* point(int obs_index) const {
    const int point_index = observations_.at<double>(obs_index, 1);
    return const_cast<double*>(&(points_.at<double>(point_index, 0)));
  }

  int camera_num() const { return camera_parameters_.rows; }
  int observations_num() const { return observations_.rows; }
  int points_num() const { return points_.rows; }

 private:
  // 観測
  // camera_id point_id u v
  cv::Mat observations_;

  // 最適化対象
  // カメラパラメータ(9 dof)
  // r p y x y z f k1 k2
  cv::Mat camera_parameters_;
  // 3D点(3 dof)
  // point_x point_y point_z
  cv::Mat points_;
};

}
}
#endif
