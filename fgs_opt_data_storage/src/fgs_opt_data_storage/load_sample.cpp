#include "fgs_opt_data_storage/loader.hpp"

#include <iostream>

using fgs::opt_data_storage::LoadCVYaml;

int main(int argc, char** argv) {

  std::cout << "argc : " << argc << std::endl;
  std::cout << "argv " << std::endl;
  for (int i = 0; i < argc; ++i) {
    std::cout << argv[i] << std::endl;
  }

  if (argc != 2) {
    std::cerr << "usage: rosrun fgs_opt_data_storage "
      "fgs_opt_data_storage_load_sample file_path" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<cv::Mat> data = LoadCVYaml(argv[1]);

//for (std::vector<cv::Mat>::const_iterator it = data.begin(); it != data.end(); ++it) {
//  std::cout << (*it) << std::endl;
//}
  std::cout << "data num : " << data.size() << std::endl;

  return EXIT_SUCCESS;
}
