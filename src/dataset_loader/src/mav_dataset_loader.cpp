#include "mav_dataset_loader.hpp"

MavDatasetLoader::MavDatasetLoader(const std::string &dataset_path) {
  dataset_path_ = dataset_path;
  if (!LoadCameraImages()) return;
  if (!LoadIMUData()) return;
}
