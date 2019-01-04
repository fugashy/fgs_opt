#ifndef FGS_CERES_PLAYGROUND_PARAMETER_HPP_
#define FGS_CERES_PLAYGROUND_PARAMETER_HPP_
#include <limits>
#include <memory>
#include <vector>

#include "fgs_ceres_playground/randomizer.hpp"

namespace fgs {
namespace ceres_playground {

template<int D>
struct Parameter : private std::vector<double> {
  Parameter() {
    this->resize(D);
    for (int i = 0; i < D; ++i) {
      (*this)[i] = 0.0;
    }
  }

  void Init(double scale) {
    std::vector<double>().swap(*this);
    this->resize(D);
    for (int i = 0; i < D; ++i) {
      (*this)[i] = Randomizer::GenerateByURD(scale);
    }
  }

  using std::vector<double>::operator[];
};
}
}
#endif
