#ifndef FGS_CERES_PLAYGROUND_INITIALIZER_HPP_
#define FGS_CERES_PLAYGROUND_INITIALIZER_HPP_
#include <limits>
#include <memory>
#include <vector>

namespace fgs {
namespace g2o_playground {
class Randomizer {
 public:
  static double GenerateByURD(double scale) {
    if (std::abs(scale) < std::numeric_limits<double>::epsilon()) {
      std::cerr << "zero scale is not supported." << std::endl;
      return 0.0;
    }

    std::shared_ptr<std::uniform_real_distribution<>> dist;
    if (scale > 0.0) {
      dist.reset(new std::uniform_real_distribution<>(-scale, scale));
    } else {
      dist.reset(new std::uniform_real_distribution<>(scale, -scale));
    }

    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    return (*dist)(engine);
  }
};
}
}
#endif
