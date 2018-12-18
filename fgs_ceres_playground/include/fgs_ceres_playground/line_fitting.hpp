#ifndef FGS_CERES_PLAYGROUND_LINE_FITTING_HPP_
#define FGS_CERES_PLAYGROUND_LINE_FITTING_HPP_
#include "fgs_ceres_playground/data.hpp"

namespace fgs {
namespace ceres_playground {

struct Line2dResidual {
  class Parameter : public std::vector<double> {
    public:
     Parameter() {
       this->resize(2);
       (*this)[0] = (*this)[1] = 0.0;
     }
     void Init() {
       (*this)[0] = 3.0;
       (*this)[1] = 6.0;
     }
     double a() const { return (*this)[0]; }
     double b() const { return (*this)[0]; }
  };

  explicit Line2dResidual(const Point2d<double>& p) : p_(p) {}

  template<typename T>
  bool operator() (const T* parameter, T* residual) const {
    residual[0] = p_.y - (parameter[0] * p_.x + parameter[1]);

    return true;
  }

 private:
  Point2d<double> p_;
};

}
}
#endif
