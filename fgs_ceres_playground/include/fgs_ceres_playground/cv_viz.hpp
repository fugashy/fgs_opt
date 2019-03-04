#ifndef FGS_CERES_PLAYGROUND_CV_VIZ_HPP_
#define FGS_CERES_PLAYGROUND_CV_VIZ_HPP_
#include <memory>

namespace cv {
namespace viz {
class Viz3d;
}
}

namespace fgs {
namespace ceres_playground {

class BundleAdjustmentInTheLarge;

class CVBALVisualizer {
 public:
  CVBALVisualizer(
      const std::shared_ptr<BundleAdjustmentInTheLarge>& bal, const std::string& window_name);
  ~CVBALVisualizer();

  void AddNoise(double mu, double sigma);
  void Show();

 private:
  std::shared_ptr<BundleAdjustmentInTheLarge> bal_;
  std::unique_ptr<cv::viz::Viz3d> window_;
};

}
}
#endif
