#ifndef FGS_CERES_PLAYGROUND_SIMPLE_BA_HPP_
#define FGS_CERES_PLAYGROUND_SIMPLE_BA_HPP_
#include "ceres/rotation.h"
#include "fgs_ceres_playground/data.hpp"
#include "fgs_ceres_playground/parameter.hpp"

namespace fgs {
namespace ceres_playground {

struct SimpleBAResidual {
  typedef cv::Mat DataType;

  typedef cv::Mat ParameterType;

  explicit SimpleBAResidual(const DataType& data) : data_(data) {}

  template<typename T>
  bool operator() (
      const T* const camera_param, const T* const point, T* residual) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera_param, point, p);
    // camera[3,4,5] are the translation.
    p[0] += camera_param[3];
    p[1] += camera_param[4];
    p[2] += camera_param[5];
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];
    // Apply second and fourth order radial distortion.
    const T& l1 = camera_param[7];
    const T& l2 = camera_param[8];
    T r2 = xp*xp + yp*yp;
    T distortion = 1.0 + r2  * (l1 + l2  * r2);
    // Compute final projected point position.
    const T& focal = camera_param[6];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;
    // The error is the difference between the predicted and observed position.
    residual[0] = predicted_x - data_.at<double>(0, 0);
    residual[1] = predicted_y - data_.at<double>(0, 1);
    return true;
  }

  static void ShowParam(
      const ParameterType& camera_parameters, const ParameterType& points) {
    // T.B.D.
  }

  static ceres::CostFunction* Create(const DataType& data) {
    return (new ceres::AutoDiffCostFunction<SimpleBAResidual, 2, 9, 3>(
            new SimpleBAResidual(data)));
  }

  DataType data_;
};

}
}
#endif
