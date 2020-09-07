#include "util.hpp"

bool GetImageNamesInFolder(const std::string &path, const std::string &format,
                           std::vector<std::string> &images) {
#if defined(__linux__) || defined(__APPLE__)
  return GetImageNamesInFolderUnix(path, format, images);
#endif
  return false;
}

#if defined(__linux__) || defined(__APPLE__)
bool GetImageNamesInFolderUnix(const std::string &path,
                               const std::string &format,
                               std::vector<std::string> &images) {
  struct dirent **file_list;
  int n = scandir(path.c_str(), &file_list, 0, alphasort);
  if (n < 0) {
    std::cerr << "Error: Unable to find directory " << path << std::endl;
    return false;
  } else {
    for (int i = 0; i < n; ++i) {
      std::string file_name(file_list[i]->d_name);
      if (file_name.size() > format.size() &&
          !file_name.compare(file_name.size() - format.size(), format.size(),
                             format)) {
        images.push_back(path + '/' + file_name);
      }
    }
  }

  free(file_list);
  return true;
}
#endif

#ifdef USE_VISUALIZATION
void VisualizeCamerasAndPoints(const cv::Matx33d &K,
                               const std::vector<cv::Mat> &Rs,
                               const std::vector<cv::Mat> &ts,
                               const std::vector<cv::Point3f> &points) {
  /// Create 3D windows
  cv::viz::Viz3d window("Coordinate Frame");
  window.setWindowSize(cv::Size(500, 500));
  window.setWindowPosition(cv::Point(150, 150));
  window.setBackgroundColor();  // black by default

  // Create the pointcloud
  std::cout << "Recovering points  ... ";

  // recover estimated points3d
  std::vector<cv::Vec3f> point_cloud_est;
  for (int i = 0; i < points.size(); ++i)
    point_cloud_est.push_back(cv::Vec3f(points[i]));

  std::cout << "[DONE]" << std::endl;

  /// Recovering cameras
  std::cout << "Recovering cameras ... ";

  std::vector<cv::Affine3d> path;
  for (size_t i = 0; i < Rs.size(); ++i)
    path.push_back(cv::Affine3d(Rs[i], ts[i]));

  std::cout << "[DONE]" << std::endl;

  /// Add the pointcloud
  if (point_cloud_est.size() > 0) {
    std::cout << "Rendering points   ... ";

    cv::viz::WCloud cloud_widget(point_cloud_est, cv::viz::Color::green());
    window.showWidget("point_cloud", cloud_widget);

    std::cout << "[DONE]" << std::endl;
  } else {
    std::cout << "Cannot render points: Empty pointcloud" << std::endl;
  }

  /// Add cameras
  if (path.size() > 0) {
    std::cout << "Rendering Cameras  ... ";

    window.showWidget("cameras_frames_and_lines",
                      cv::viz::WTrajectory(path, cv::viz::WTrajectory::BOTH,
                                           0.1, cv::viz::Color::green()));
    window.showWidget(
        "cameras_frustums",
        cv::viz::WTrajectoryFrustums(path, K, 0.1, cv::viz::Color::yellow()));

    window.setViewerPose(path[0]);

    std::cout << "[DONE]" << std::endl;
  } else {
    std::cout << "Cannot render the cameras: Empty path" << std::endl;
  }

  /// Wait for key 'q' to close the window
  std::cout << std::endl
            << "Press 'q' to close each windows ... " << std::endl;

  window.spin();
}

void VisualizeCamerasAndPoints(
    const cv::Matx33d &K, const std::vector<cv::Mat> &Rs,
    const std::vector<cv::Mat> &ts,
    const std::vector<std::vector<cv::Point3f> > &points) {
  /// Create 3D windows
  cv::viz::Viz3d window("Coordinate Frame");
  window.setWindowSize(cv::Size(500, 500));
  window.setWindowPosition(cv::Point(150, 150));
  window.setBackgroundColor();  // black by default

  // Create the pointcloud
  std::cout << "Recovering points  ... ";

  for (int set_id = 0; set_id < points.size(); ++set_id) {
    std::vector<cv::Vec3f> point_cloud_est;
    for (int i = 0; i < points[set_id].size(); ++i)
      point_cloud_est.push_back(cv::Vec3f(points[set_id][i]));
    if (point_cloud_est.size() > 0) {
      cv::viz::Color point_color;
      switch (set_id) {
        case 0:
          point_color = cv::viz::Color::green();
          break;
        case 1:
          point_color = cv::viz::Color::red();
          break;
        case 2:
          point_color = cv::viz::Color::cherry();
          break;
        case 3:
          point_color = cv::viz::Color::orange();
          break;
        case 4:
          point_color = cv::viz::Color::pink();
          break;
        case 5:
          point_color = cv::viz::Color::blue();
          break;
        default:
          std::cout << "More than 6 sets of points.\n";
          point_color = cv::viz::Color::yellow();
      }
      cv::viz::WCloud cloud_widget(point_cloud_est, point_color);
      std::string cloud_name = "point_cloud" + std::to_string(set_id);
      window.showWidget(cloud_name, cloud_widget);
    } else {
      std::cout << "Cannot render points: Empty pointcloud" << std::endl;
    }
  }
  /// Recovering cameras
  std::cout << "Recovering cameras ... ";

  std::vector<cv::Affine3d> path;
  for (size_t i = 0; i < Rs.size(); ++i)
    path.push_back(cv::Affine3d(Rs[i], ts[i]));

  std::cout << "[DONE]" << std::endl;

  /// Add the pointcloud
  /// Add cameras
  if (path.size() > 0) {
    std::cout << "Rendering Cameras  ... ";

    window.showWidget("cameras_frames_and_lines",
                      cv::viz::WTrajectory(path, cv::viz::WTrajectory::BOTH,
                                           0.1, cv::viz::Color::green()));
    window.showWidget(
        "cameras_frustums",
        cv::viz::WTrajectoryFrustums(path, K, 0.1, cv::viz::Color::yellow()));

    window.setViewerPose(path[0]);

    std::cout << "[DONE]" << std::endl;
  } else {
    std::cout << "Cannot render the cameras: Empty path" << std::endl;
  }

  /// Wait for key 'q' to close the window
  std::cout << std::endl
            << "Press 'q' to close each windows ... " << std::endl;

  window.spin();
}

#endif
