#ifndef FGS_CERES_PLAYGROUND_PARAMETER_HPP_
#define FGS_CERES_PLAYGROUND_PARAMETER_HPP_
#include <limits>
#include <memory>
#include <vector>

namespace fgs {
namespace ceres_playground {

template<int D>
struct Parameter : public std::vector<double> {
  Parameter() {
    this->resize(D);
    for (int i = 0; i < D; ++i) {
      (*this)[i] = 0.0;
    }
  }

  void Init(double scale) {
    if (std::abs(scale) < std::numeric_limits<double>::epsilon()) {
      std::cerr << "zero is not supported." << std::endl;
      return;
    }

    std::shared_ptr<std::uniform_real_distribution<>> dist;
    if (scale > 0.0) {
      dist.reset(new std::uniform_real_distribution<>(-scale, scale));
    } else {
      dist.reset(new std::uniform_real_distribution<>(scale, -scale));
    }

    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    for (int i = 0; i < D; ++i) {
      (*this)[i] = (*dist)(engine);
    }
  }
};
}
}
#endif
