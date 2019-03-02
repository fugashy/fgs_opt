#include <iostream>
#include "fgs_opt_data_storage/loader.hpp"
#include <sstream>

namespace fgs {
namespace opt_data_storage {

std::vector<cv::Mat> LoadCVYaml(const std::string& file_path) {
  cv::FileStorage fs(file_path, cv::FileStorage::READ);

  std::vector<cv::Mat> return_data;

  // read data
  cv::Mat data;
  fs["data"] >> data;
  return_data.push_back(data);

  // read extended data
  // this is for problems such as bundle-adjustment
  uint32_t i = 0;
  while (true) {
    std::stringstream ss;
    ss << "extend_data" << i;
    cv::Mat extend_data;
    fs[ss.str()] >> extend_data;
    if (extend_data.empty()) {
      return return_data;
    }
    return_data.push_back(data);
    ++i;
  }
  return return_data;
}

}
}
