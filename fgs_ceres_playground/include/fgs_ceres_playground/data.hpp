#ifndef FGS_CERES_PLAYGROUND_DATA_HPP_
#define FGS_CERES_PLAYGROUND_DATA_HPP_

namespace fgs {
namespace ceres_playground {

template<typename T>
struct Point2d {
  T x;
  T y;
};

template<typename T>
struct Point3d {
  T x;
  T y;
  T z;
};

}
}
#endif
