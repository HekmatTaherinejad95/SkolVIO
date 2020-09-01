#include <iostream>
#include <string>

#include "camera_model.hpp"
#include "util.hpp"

using namespace std;

class Options {
 public:
  Options() {}
  std::string path;
  std::string format;
};

int main(int argc, char **argv) {
  Options option;
  for (int i = 0; i < argc; ++i) {
    if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--path")) {
      option.path = argv[++i];
      // Remove trailing '/'
      while (option.path.back() == '/') option.path.pop_back();
    } else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--format")) {
      option.format = argv[++i];
    }
  }

  if (!option.format.size() || !option.path.size()) {
    cout << "Error. Unknown arguments.\n";
    cout << "Usage: \n";
    cout << "       camera_model_app\n";
    cout << "            -p, --path full_path \n";
    cout << "            -f, --format image format, e.g png, jpg\n";
    cout << "Exampe: \n";
    cout << "./camera_model_app -p ~/Project/vio/data/desk_subset/ -f jpg\n";
    return -1;
  }

  const std::string config_file_name = "calibration.yaml";
  const std::string full_path_to_config = option.path + '/' + config_file_name;
  cv::FileStorage config_file;
  config_file.open(full_path_to_config, cv::FileStorage::READ);
  if (!config_file.isOpened()) {
    std::cerr << "Error: Couldn't open config file " << full_path_to_config
              << ".\n";
    return -1;
  }

  auto camera_ptr =
      vio::CreateCameraModelFromConfig(config_file["CameraModel"]);
  if (camera_ptr == nullptr) {
    std::cerr << "Error: Couldn't create camera model.\n";
    return -1;
  }

  vector<string> images;
  if (!GetImageNamesInFolder(option.path, option.format, images)) {
    return -1;
  }

  cv::namedWindow("original", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("undistorted", cv::WINDOW_AUTOSIZE);
  for (int i = 0; i < images.size(); ++i) {
    cv::Mat image = cv::imread(images[i]);
    cv::Mat undistorted_image;
    camera_ptr->UndistortImage(image, undistorted_image);

    cv::imshow("original", image);
    cv::imshow("undistorted", undistorted_image);
    cv::waitKey(0);
  }
  return 0;
}
