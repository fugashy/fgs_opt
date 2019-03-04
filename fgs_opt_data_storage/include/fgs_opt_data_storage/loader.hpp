#ifndef FGS_OPT_DATA_STORAGE_LOADER_HPP_
#define FGS_OPT_DATA_STORAGE_LOADER_HPP_
#include <vector>

#include <opencv2/core.hpp>

namespace fgs {
namespace opt_data_storage {

std::vector<cv::Mat> LoadCVYaml(const std::string& file_path);

}
}
#endif
