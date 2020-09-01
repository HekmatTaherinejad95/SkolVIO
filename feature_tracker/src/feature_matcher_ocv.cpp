#include "feature_matcher_ocv.hpp"

#include <iostream>

#include "../../util/include/timer.hpp"

namespace vio {

std::unique_ptr<FeatureMatcher> FeatureMatcher::CreateFeatureMatcherOCV(
    const FeatureMatcherOptions &option) {
  return std::unique_ptr<FeatureMatcher>(new FeatureMatcherOCV(option));
}

FeatureMatcherOCV::FeatureMatcherOCV(const FeatureMatcherOptions &option)
    : FeatureMatcher(option) {
  // TODO: Decide matcher based on descriptors
  // Hamming-distance works only for binary feature-types like ORB, FREAK
  if (option.desc_dist_type == FeatureMatcherOptions::HAMMING) {
    matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::cout << "Created OCV Matcher BruteForce-Hamming.\n";
  } else if (option.desc_dist_type == FeatureMatcherOptions::NORM_L2) {
    matcher_ = cv::DescriptorMatcher::create("BruteForce");
    std::cout << "Created OCV Matcher " << option.ocv_matcher_type << std::endl;
  }

  // matcher_ = cv::DescriptorMatcher::create("FlannBased");
}

bool FeatureMatcherOCV::Match(const ImageFrame &frame0,
                              const ImageFrame &frame1,
                              std::vector<cv::DMatch> &matches) {
  const std::vector<cv::KeyPoint> &kp0 = frame0.keypoints();
  const std::vector<cv::KeyPoint> &kp1 = frame1.keypoints();
  const cv::Mat &desc0 = frame0.descriptors();
  const cv::Mat &desc1 = frame1.descriptors();

  Timer timer;
  timer.Start();

  std::vector<std::vector<cv::DMatch> > matches_0to1_k, matches_1to0_k;
  matcher_->knnMatch(desc0, desc1, matches_0to1_k, max_match_per_desc_);
  matcher_->knnMatch(desc1, desc0, matches_1to0_k, max_match_per_desc_);

  //  matcher_->radiusMatch(desc0, desc1, matches_0to1_k, 20);
  //  matcher_->radiusMatch(desc1, desc0, matches_1to0_k, 20);

  timer.Stop();
  // std::cout << "Knn match time used: " << timer.GetInMs() << "ms.\n";
  timer.Start();

  // Pick matches where the first one is much better than the second match.
  std::vector<cv::DMatch> matches_0to1, matches_1to0;
  RatioTestFilter(matches_0to1_k, matches_0to1);
  RatioTestFilter(matches_1to0_k, matches_1to0);

  timer.Stop();
  // std::cout << "Ratio test time used: " << timer.GetInMs() << "ms.\n";
  timer.Start();

  if (matches_0to1.size() < 10 || matches_1to0.size() < 10) {
    std::cerr << "Error: Not enough matches after ratio test.\n";
    std::cerr << "Match 0 to 1: " << matches_0to1.size() << " / "
              << matches_0to1_k.size() << std::endl;
    std::cerr << "Match 1 to 0: " << matches_1to0.size() << " / "
              << matches_1to0_k.size() << std::endl;
    return false;
  }
  // matches is pre to cur
  SymmetryTestFilter(matches_0to1, matches_1to0, matches);
  if (matches.size() < 5) {
    std::cerr << "Error: Not enough matches after symmetry test.\n";
    return false;
  }

  timer.Stop();
  // std::cout << "Symmetry test time used: " << timer.GetInMs() << "ms.\n";
  timer.Start();

  RemoveOutlierMatch(kp0, kp1, matches);

  timer.Stop();
  // std::cout << "F matrix outlier test time used: " << timer.GetInMs()
  //          << "ms.\n";

  if (matches.size() < 3) {
    std::cerr << "Error: Not enough matches after outlier removal.\n";
    return false;
  }

  return true;
}

}  // vio
