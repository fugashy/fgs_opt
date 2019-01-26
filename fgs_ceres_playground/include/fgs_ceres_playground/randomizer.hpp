#ifndef FGS_CERES_PLAYGROUND_INITIALIZER_HPP_
#define FGS_CERES_PLAYGROUND_INITIALIZER_HPP_
#include <limits>
#include <memory>
#include <vector>

namespace fgs {
namespace ceres_playground {
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

  static double GenerateByND(double mean, double stddev) {
    if (stddev < std::numeric_limits<double>::epsilon()) {
      std::cerr << " scale is not supported." << std::endl;
      return 0.0;
    }
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());

    std::normal_distribution<> dist(mean, stddev);
    return dist(engine);
  }
};
}
}
#endif
