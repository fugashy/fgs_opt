#include <iostream>
#include "fgs_opt_data_storage/loader.hpp"

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
  cv::FileNode ed_fn = fs["extended_data"];
  if (ed_fn.empty()) {
    return return_data;
  }
  if (ed_fn.type() != cv::FileNode::SEQ) {
    throw std::runtime_error("extended_data should be sequential type");
  }
  for (cv::FileNodeIterator it = ed_fn.begin(); it != ed_fn.end(); ++it) {
    cv::Mat data;
    (*it)["data"] >> data;
    return_data.push_back(data);
  }
  return return_data;
}

}
}
