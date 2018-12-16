#ifndef FGS_OPT_DATA_STORAGE_LOADER_HPP_
#define FGS_OPT_DATA_STORAGE_LOADER_HPP_
#include <opencv2/core.hpp>

namespace fgs {
namespace opt_data_storage {

cv::Mat LoadCVYaml(const std::string& file_path);

}
}
#endif
