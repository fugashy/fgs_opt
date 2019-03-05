#ifndef FGS_G2O_PLAYGROUND_LINE_HPP_
#define FGS_G2O_PLAYGROUND_LINE_HPP_
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"

#include "fgs_g2o_playground/randomizer.hpp"

namespace fgs {
namespace g2o_playground {

// y = a * x + b
// parameter: a, b
class LineParameter : public g2o::BaseVertex<2,              /* num of param  */
                                             Eigen::Vector2d /* type of param */> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  LineParameter() {}

  virtual bool read(std::istream&) {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }

  virtual bool write(std::ostream&) const {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }

  virtual void setToOriginImpl() {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
  }

  virtual void oplusImpl(const double* update) {
    Eigen::Vector2d::ConstMapType v(update);
    _estimate += v;
  }

  void Init(double scale) {
    const double a = Randomizer::GenerateByURD(scale);
    const double b = Randomizer::GenerateByURD(scale);
    const Eigen::Vector2d estimate(a, b);
    this->setEstimate(estimate);
  }

  void ShowParam() {
    std::cout << '\t' << "a: "      << this->estimate()(0) << std::endl
              << '\t' << "b: "      << this->estimate()(1) << std::endl;
  }
};

class LineData : public g2o::BaseUnaryEdge<1,               /* dim of residual */
                                           Eigen::Vector2d, /* type of data    */
                                           LineParameter    /* type of param   */> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  LineData() {}

  virtual bool read(std::istream& is) {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }

  virtual bool write(std::ostream& is) const {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }

  void computeError() {
    const LineParameter* params = static_cast<const LineParameter*>(vertex(0));

    const double& a = params->estimate()(0);
    const double& b = params->estimate()(1);

    double fx = a * measurement()(0) + b;

    _error(0) = measurement()(1) - fx;
  }

  void SetData(const cv::Mat& data) {
    Eigen::Vector2d measurement;
    measurement[0] = data.at<double>(0, 0);
    measurement[1] = data.at<double>(0, 1);
    this->setMeasurement(measurement);
    this->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
  }

};

class Line {
 public:
  typedef LineData DataType;
  typedef LineParameter ParameterType;
};

}
}
#endif
