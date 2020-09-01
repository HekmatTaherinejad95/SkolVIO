#include "feature_matcher_grid_search.hpp"

#include "../../util/include/timer.hpp"

namespace vio {

std::unique_ptr<FeatureMatcher> FeatureMatcher::CreateFeatureMatcherGridSearch(
    const FeatureMatcherOptions &option) {
  return std::unique_ptr<FeatureMatcher>(new FeatureMatcherGridSearch(option));
}

FeatureMatcherGridSearch::FeatureMatcherGridSearch(
    const FeatureMatcherOptions &option)
    : FeatureMatcher(option), pixel_search_range_(option.pixel_search_range) {
  std::cout << "Created Grid Search Matcher with distance type: ";
  if (dist_type_ == FeatureMatcherOptions::HAMMING)
    std::cout << "Hamming.\n";
  else
    std::cout << "Norm_L2.\n";
}

bool FeatureMatcherGridSearch::Match(const ImageFrame &frame0,
                                     const ImageFrame &frame1,
                                     std::vector<cv::DMatch> &matches) {
  std::vector<std::vector<cv::DMatch> > matches_0to1_k, matches_1to0_k;
  Timer timer;
  timer.Start();

  FindMatchNearFeatures(frame0, frame1, matches_0to1_k);
  FindMatchNearFeatures(frame1, frame0, matches_1to0_k);

  timer.Stop();
  // std::cout << "Grid Search Match time used: " << timer.GetInMs() << "ms.\n";
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

  RemoveOutlierMatch(frame0.keypoints(), frame1.keypoints(), matches);
  // RemoveOutlierMatch(kp0, kp1, matches);

  timer.Stop();
  // std::cout << "F matrix outlier test time used: " << timer.GetInMs()
  //          << "ms.\n";

  if (matches.size() < 3) {
    std::cerr << "Error: Not enough matches after outlier removal.\n";
    return false;
  }

  return true;
}

bool FeatureMatcherGridSearch::FindMatchNearFeatures(
    const ImageFrame &query_frame, const ImageFrame &ref_frame,
    std::vector<std::vector<cv::DMatch> > &matches) {
  const std::vector<cv::KeyPoint> &kp0 = query_frame.keypoints();
  const cv::Mat &desc0 = query_frame.descriptors();
  const cv::Mat &desc1 = ref_frame.descriptors();

  matches.clear();
  for (int i = 0; i < kp0.size(); ++i) {
    // For each keypoint find matches.
    const cv::KeyPoint &kp = kp0[i];
    std::vector<int> near_f_id;
    // TODO: Make threshold argument.
    if (!ref_frame.GetNeighborKeypointsInRadius(kp, pixel_search_range_,
                                                near_f_id))
      return false;  // This means fail. Not just couldn't find matches.
    // Not enough matches, skip.
    if (near_f_id.size() < 2) continue;

    // Find best 2 matches.
    double best_dist[2];
    int best_id[2];
    double dist0 = ComputeDistance(desc0.row(i), desc1.row(near_f_id[0]));
    double dist1 = ComputeDistance(desc0.row(i), desc1.row(near_f_id[1]));

    if (dist0 < dist1) {
      best_dist[0] = dist0;
      best_dist[1] = dist1;
      best_id[0] = 0;
      best_id[1] = 1;
    } else {
      best_dist[1] = dist0;
      best_dist[0] = dist1;
      best_id[1] = 0;
      best_id[0] = 1;
    }

    for (int j = 2; j < near_f_id.size(); ++j) {
      double dist = ComputeDistance(desc0.row(i), desc1.row(near_f_id[j]));
      if (dist < best_dist[0]) {
        best_dist[1] = best_dist[0];
        best_dist[0] = dist;
        best_id[1] = best_id[0];
        best_id[0] = j;
      } else if (dist < best_dist[1]) {
        best_dist[1] = dist;
        best_id[1] = j;
      }
    }
    std::vector<cv::DMatch> match;
    match.push_back(cv::DMatch(i, near_f_id[best_id[0]], best_dist[0]));
    match.push_back(cv::DMatch(i, near_f_id[best_id[1]], best_dist[1]));
    matches.push_back(match);
  }
  return true;
}

inline double FeatureMatcherGridSearch::ComputeDistance(const cv::Mat &mat0,
                                                        const cv::Mat &mat1) {
  if (dist_type_ == FeatureMatcherOptions::NORM_L2)
    return cv::norm(mat0, mat1, cv::NORM_L2);
  else if (dist_type_ == FeatureMatcherOptions::HAMMING)
    return cv::norm(mat0, mat1, cv::NORM_HAMMING);
  return -1;
}

}  // vio
