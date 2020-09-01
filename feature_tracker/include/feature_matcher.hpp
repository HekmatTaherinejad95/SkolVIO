#ifndef VIO_FEATURE_MATCHER_
#define VIO_FEATURE_MATCHER_

#include <iostream>
#include <string>
#include <memory>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "image_frame.hpp"

namespace vio {

enum FeatureMatcherMethod { OCV = 0, GRID_SEARCH };

class FeatureMatcherOptions {
 public:
  FeatureMatcherOptions()
      : method(GRID_SEARCH),
        desc_dist_type(HAMMING),
        ocv_matcher_type("BruteForce"),
        use_ratio_test(true),
        ratio_test_thresh(0.9),
        use_symmetry_test(true),
        use_remove_outliers(true),
        max_dist_to_epipolar_line(0.5),
        level_of_confidence(0.999),
        pixel_search_range(30) {}

  enum FeatureMatcherMethod { OCV = 0, GRID_SEARCH };
  enum FeatureMatcherDistType { HAMMING = 0, NORM_L2 };

  bool use_ratio_test;
  double ratio_test_thresh;

  bool use_symmetry_test;

  bool use_remove_outliers;
  double max_dist_to_epipolar_line;
  double level_of_confidence;

  FeatureMatcherMethod method;
  // Distance type, haming or l2
  FeatureMatcherDistType desc_dist_type;

  // FeatureMatcherOCV
  std::string ocv_matcher_type;

  double pixel_search_range;

  void read(const cv::FileNode &node) {
    method = static_cast<FeatureMatcherMethod>((int)node["Method"]);
    max_dist_to_epipolar_line = (double)node["MaxDistToEpipolarLine"];
    level_of_confidence = (double)node["LevelOfConfidence"];

    if (method == OCV) {
      std::cout << "Selected OpenCV FeatureMatcher.\n";

      ocv_matcher_type = (std::string)node["OCVMatcherType"];
      std::cout << "Selected matcher type: " << ocv_matcher_type << std::endl;

    } else if (method == GRID_SEARCH) {
      std::cout << "Selected Grid Search FeatureMatcher.\n";
      desc_dist_type =
          static_cast<FeatureMatcherDistType>((int)node["DistType"]);
      pixel_search_range = (double)node["PixelSearchRange"];
    }
  }
};

// Following must be defined for the serialization in FileStorage to work
static void read(
    const cv::FileNode &node, FeatureMatcherOptions &x,
    const FeatureMatcherOptions &default_value = FeatureMatcherOptions()) {
  if (node.empty())
    x = default_value;
  else
    x.read(node);
}

class FeatureMatcher {
 public:
  explicit FeatureMatcher(const FeatureMatcherOptions &option)
      : max_match_per_desc_(2),
        max_dist_to_epipolar_line_(option.max_dist_to_epipolar_line),
        level_of_confidence_(option.level_of_confidence),
        nn_match_ratio_(option.ratio_test_thresh),
        dist_type_(option.desc_dist_type) {}

  // Call from outside.
  static std::unique_ptr<FeatureMatcher> CreateFeatureMatcher(
      const FeatureMatcherOptions &option);

  // Create different type of matcher.
  static std::unique_ptr<FeatureMatcher> CreateFeatureMatcherOCV(
      const FeatureMatcherOptions &option);
  static std::unique_ptr<FeatureMatcher> CreateFeatureMatcherGridSearch(
      const FeatureMatcherOptions &option);

  // TODO: Consider change to const.
  virtual bool Match(const ImageFrame &frame0, const ImageFrame &frame1,
                     std::vector<cv::DMatch> &matches) = 0;

 protected:
  // TODO: Right now, it's O(n^2) search time.
  bool SymmetryTestFilter(const std::vector<cv::DMatch> &matches1,
                          const std::vector<cv::DMatch> &matches2,
                          std::vector<cv::DMatch> &final_matches);
  bool RatioTestFilter(std::vector<std::vector<cv::DMatch> > best_k,
                       std::vector<cv::DMatch> &matches);

  bool RemoveOutlierMatch(const std::vector<cv::KeyPoint> &pre_kp,
                          const std::vector<cv::KeyPoint> &cur_kp,
                          std::vector<cv::DMatch> &matches);

  int max_match_per_desc_;
  double max_dist_to_epipolar_line_;
  double level_of_confidence_;
  double nn_match_ratio_;
  FeatureMatcherOptions::FeatureMatcherDistType dist_type_;
};

typedef std::shared_ptr<FeatureMatcher> FeatureMatcherPtr;

}  // vio

#endif
