#include "camera_model.hpp"

namespace vio {

CameraModelPtr CreateCameraModelFromConfig(const cv::FileNode &node) {
  const std::string camera_type = (std::string)node["CameraType"];
  const int num_params = (int)node["NumParams"];
  const int image_width = (int)node["ImageWidth"];
  const int image_height = (int)node["ImageHeight"];
  // Load parameters.
  cv::Mat params_cv;
  node["Params"] >> params_cv;

  if (camera_type == "Pinhole") {
    std::cout << "Creating Pinhole model...\n";
    if (num_params != PINHOLE_NUM_PARAMETERS ||
        params_cv.cols * params_cv.rows != PINHOLE_NUM_PARAMETERS) {
      std::cerr << "Error: Number of parameters doesn't match.\n";
      return nullptr;
    }

    PinholeCameraModel<double>::ParamsArray params;
    for (int i = 0; i < num_params; ++i) {
      params[i] = params_cv.at<double>(i);
    }

    std::unique_ptr<vio::CameraModel> camera =
        std::unique_ptr<vio::CameraModel>(new vio::PinholeCameraModel<double>(
            image_height, image_width, params));
    std::cout << "Loaded camera matrix:\n" << camera->camera_matrix()
              << std::endl;
    return std::move(camera);
  } else {
    std::cerr << "Error: Unknown camera model name <" << camera_type << ">.\n";
    return nullptr;
  }
}

}  // namespace vio
