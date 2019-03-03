#include "fgs_ceres_playground/cv_viz.hpp"

#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include "fgs_ceres_playground/bundle_adjustment_in_the_large.hpp"


namespace fgs {
namespace ceres_playground {

CVBALVisualizer::CVBALVisualizer(
    BundleAdjustmentInTheLarge& bal, const std::string& window_name) :
  bal_(bal), window_(new cv::viz::Viz3d(window_name)) {
}

CVBALVisualizer::~CVBALVisualizer() {
  window_->removeAllWidgets();
}

void CVBALVisualizer::AddNoise(double mu, double sigma) {
  std::mt19937 rand_src(12345);
  for (int i = 0; i < bal_.points_num(); ++i) {
    std::normal_distribution<double> rand_dist(mu, sigma);
    bal_.point(0).at<double>(0) += rand_dist(rand_src);
    bal_.point(0).at<double>(1) += rand_dist(rand_src);
    bal_.point(0).at<double>(2) += rand_dist(rand_src);
  }
}

void CVBALVisualizer::Show() {
  for (int i = 0; i < bal_.camera_num(); i++) {
    cv::Vec3d rot = cv::Vec3d(bal_.camera(i)(cv::Rect(0, 0, 0, 2)));
    cv::Vec3d tvec = cv::Vec3d(bal_.camera(i)(cv::Rect(0, 3, 0, 5)));
    cv::Affine3d camera_pose = cv::Affine3d(rot, tvec);

    // Coordinate axes
    cv::viz::WCameraPosition cpw(-0.1);
    // Camera frustum
    cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), -0.1);

    std::string widgetPoseName = "CPW" + std::to_string(i);
    std::string widgetFrustumName = "CPW_FRUSTUM" + std::to_string(i);
    window_->showWidget(widgetPoseName, cpw, camera_pose);
    window_->showWidget(widgetFrustumName, cpw_frustum, camera_pose);
  }

  std::vector<cv::Vec3d> points3d(bal_.points_num());
  for (int i = 0; i < bal_.points_num(); ++i) {
    const cv::Vec3d point3d(bal_.point(i));
    points3d[i] = point3d;
  }
  cv::viz::WCloud wcloud(points3d, cv::viz::Color(0, 0, 255));

  window_->showWidget("points", wcloud);
  window_->spinOnce(1, true);

  return;
}
}
}
