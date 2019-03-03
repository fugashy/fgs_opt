#ifndef FGS_CERES_PLAYGROUND_BUNDLE_ADJUSTMENT_IN_THE_LARGE_HPP_
#define FGS_CERES_PLAYGROUND_BUNDLE_ADJUSTMENT_IN_THE_LARGE_HPP_
#include <string>

#include <opencv2/core.hpp>

namespace fgs {
namespace ceres_playground {

// Bundle Adjustment in the Large データセットを読み込む
// 残差ブロック定義時に便利になるようなアクセス方法を提供する
class BundleAdjustmentInTheLarge {
 explicit BundleAdjustmentInTheLarge(const std::string& cv_storage_path);

 // 観測画像点
 // camera_idとpoint_idを飛ばしてアクセスしたいので列番号は2
 const double* observation(int i) const { return &(observations_.at<double>(i, 2)); }

 // 観測画像点に対応したカメラパラメータへのアドレス
 const double* accociated_camera_parameter(int i) const {
   const int index = observations_.at<double>(4*i, 0);
   return &(camera_parameters_.at<double>(index, 0));
 }
 // 観測画像点に対応した点へのアドレス
 const double* accosiated_point(int i) const {
   const int index = observations_.at<double>(4*i, 1);
   return &(points_.at<double>(index, 0));
 }

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
