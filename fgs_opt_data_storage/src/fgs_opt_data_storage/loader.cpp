#include "fgs_opt_data_storage/loader.hpp"

namespace fgs {
namespace opt_data_storage {

cv::Mat LoadCVYaml(const std::string& file_path) {
  cv::FileStorage fs(file_path, cv::FileStorage::READ);

  cv::Mat out;
  fs["data"] >> out;

  return out;
}

}
}
