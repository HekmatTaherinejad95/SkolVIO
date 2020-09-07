#include <dirent.h>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#ifdef USE_VISUALIZATION
#include <opencv2/viz.hpp>
#endif

bool GetImageNamesInFolder(const std::string &path, const std::string &format,
                           std::vector<std::string> &images);

#if defined(__linux__) || defined(__APPLE__)
bool GetImageNamesInFolderUnix(const std::string &path,
                               const std::string &format,
                               std::vector<std::string> &images);
#endif

#ifdef USE_VISUALIZATION
void VisualizeCamerasAndPoints(const cv::Matx33d &K,
                               const std::vector<cv::Mat> &Rs,
                               const std::vector<cv::Mat> &ts,
                               const std::vector<cv::Point3f> &points);

// Allow plot multiple set of points with different color.
void VisualizeCamerasAndPoints(
    const cv::Matx33d &K, const std::vector<cv::Mat> &Rs,
    const std::vector<cv::Mat> &ts,
    const std::vector<std::vector<cv::Point3f> > &points);
#endif
