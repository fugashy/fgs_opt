#ifndef FGS_G2O_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_
#define FGS_G2O_PLAYGROUND_OPTIMIZATION_CONTEXT_HPP_


#include <opencv2/core.hpp>

#include "fgs_g2o_playground/curve.hpp"
#include "fgs_g2o_playground/line.hpp"
#include "fgs_g2o_playground/optimizer.hpp"

namespace fgs {
namespace g2o_playground {

class OptimizationContext {
 public:
  virtual void Optimize() = 0;
};

template<class T>
class FittingContext : public OptimizationContext {
 public:
  explicit FittingContext(const std::string& cv_storage_path) {
    cv::FileStorage fs(cv_storage_path, cv::FileStorage::READ);
    cv::Mat data_mat;
    fs["data"] >> data_mat;
    if (data_mat.empty()) {
      throw std::runtime_error("data is empty");
    }

    data_array_.resize(data_mat.rows);

    for (int i = 0, iend = data_array_.size(); i < iend; ++i) {
      typename T::DataType* data(new typename T::DataType());
      data->SetData(data_mat.row(i));
      data_array_[i] = data;
    }
  }

  virtual void Optimize() {
    typename T::ParameterType* param(new typename T::ParameterType());

    std::cout << "Before" << std::endl;
    param->Init(100.0);
    param->ShowParam();
    param->setId(0);

    auto optimizer = Optimizer::Create("levenberg");
    optimizer->addVertex(param);

    for (int i = 0, iend = data_array_.size(); i < iend; ++i) {
      data_array_[i]->setVertex(0, param);
      optimizer->addEdge(data_array_[i]);
    }

    optimizer->initializeOptimization();
    optimizer->setVerbose(true);
    optimizer->optimize(1000);

    std::cout << "After" << std::endl;
    param->ShowParam();
  }

 private:
  std::vector<typename T::DataType*> data_array_;
};

}
}
#endif
