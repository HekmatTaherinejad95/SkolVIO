#include "feature_matcher.hpp"

#include <unordered_map>

namespace vio {

std::unique_ptr<FeatureMatcher> FeatureMatcher::CreateFeatureMatcher(
    const FeatureMatcherOptions &option) {
  switch (option.method) {
    case OCV:
      return CreateFeatureMatcherOCV(option);
    case GRID_SEARCH:
      return CreateFeatureMatcherGridSearch(option);
    default:
      return nullptr;
  }
}

bool FeatureMatcher::SymmetryTestFilter(
    const std::vector<cv::DMatch> &matches1,
    const std::vector<cv::DMatch> &matches2,
    std::vector<cv::DMatch> &final_matches) {
  final_matches.clear();

  std::unordered_map<int, int> match_map;
  for (std::vector<cv::DMatch>::const_iterator matchIterator = matches1.begin();
       matchIterator != matches1.end(); ++matchIterator) {
    match_map[(*matchIterator).trainIdx] = (*matchIterator).queryIdx;
  }

  for (std::vector<cv::DMatch>::const_iterator matchIterator = matches2.begin();
       matchIterator != matches2.end(); ++matchIterator) {
    auto match_ptr = match_map.find((*matchIterator).queryIdx);
    if (match_ptr != match_map.end() &&
        match_ptr->second == (*matchIterator).trainIdx) {
      // Here need to reverse trainIdx and queryIdx to suit matches1
      final_matches.push_back(cv::DMatch((*matchIterator).trainIdx,
                                         (*matchIterator).queryIdx,
                                         (*matchIterator).distance));
    }
  }
  return true;
}

bool FeatureMatcher::RatioTestFilter(
    std::vector<std::vector<cv::DMatch> > best_k,
    std::vector<cv::DMatch> &matches) {
  for (int i = 0; i < best_k.size(); ++i) {
    if (best_k[i][0].distance < nn_match_ratio_ * best_k[i][1].distance) {
      matches.push_back(best_k[i][0]);
    }
  }
  return true;
}

bool FeatureMatcher::RemoveOutlierMatch(const std::vector<cv::KeyPoint> &pre_kp,
                                        const std::vector<cv::KeyPoint> &cur_kp,
                                        std::vector<cv::DMatch> &matches) {
  if (matches.size() < 8) {
    std::cout << "Skipped Remove Outlier. Only " << matches.size();
    std::cout << " pairs of matches are provided.\n";
    // TODO: Does this function need to return value?
    return true;
  }
  // TODO: Check the speed.
  std::vector<cv::Point2f> pre_matched_kp, cur_matched_kp;
  for (int i = 0; i < matches.size(); ++i) {
    pre_matched_kp.push_back(pre_kp[matches[i].queryIdx].pt);
    cur_matched_kp.push_back(cur_kp[matches[i].trainIdx].pt);
  }
  cv::Mat mask;
  // TODO: Need to tune the parameters, e.g. 3
  // TODO: Normalize
  cv::Mat fundamental_matrix = cv::findFundamentalMat(
      pre_matched_kp, cur_matched_kp, cv::FM_RANSAC, max_dist_to_epipolar_line_,
      level_of_confidence_, mask);
  int num_outlier = 0;
  std::vector<cv::DMatch> new_matches;
  for (int i = 0; i < matches.size(); ++i) {
    if ((unsigned int)mask.at<uchar>(i)) new_matches.push_back(matches[i]);
  }
  // std::cout << "Outlier matches: " << matches.size() - new_matches.size()
  //          << std::endl;

  matches = std::move(new_matches);
  return true;
}

}  // vio
