#ifndef FGS_CERES_PLAYGROUND_DATA_HPP_
#define FGS_CERES_PLAYGROUND_DATA_HPP_

namespace fgs {
namespace ceres_playground {

struct Point2d : public std::vector<double> {
  Point2d() {
    this->resize(2);
    (*this)[0] = (*this)[1] = 0.0;
  }
  Point2d(double x, double y) {
    this->resize(2);
    (*this)[0] = x;
    (*this)[1] = y;
  }

  void x(double x) { (*this)[0] = x; }
  void y(double y) { (*this)[1] = y; }

  double get_x() const { return (*this)[0]; }
  double get_y() const { return (*this)[1]; }
};

}
}
#endif
