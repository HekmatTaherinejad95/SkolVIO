#include "feature_tracker_ocv.hpp"

#include <iostream>

#include "../../util/include/timer.hpp"
#include "feature_matcher_grid_search.hpp"
#include "feature_matcher_ocv.hpp"

namespace vio {

std::unique_ptr<FeatureTracker> FeatureTracker::CreateFeatureTrackerOCV(
    FeatureTrackerOptions option, std::unique_ptr<FeatureMatcher> matcher) {
  switch (option.method) {
    case FeatureTrackerOptions::OCV_BASIC_DETECTOR:
    case FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR:
      return std::unique_ptr<FeatureTracker>(
          new FeatureTrackerOCV(option, std::move(matcher)));
    default:
      return nullptr;
  }
}

FeatureTrackerOCV::FeatureTrackerOCV(FeatureTrackerOptions option,
                                     std::unique_ptr<FeatureMatcher> matcher)
    : detector_type_(DETECTORONLY),
      max_feature_per_frame_(option.max_num_feature) {
  if (option.detector_type == "ORB") {
    // Parameters:
    // int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int
    // edgeThreshold=31,
    // int firstLevel=0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE,
    // int patchSize=31, int fastThreshold=20
    detector_ = cv::ORB::create(max_feature_per_frame_);
    std::cout << "Created ORB Detector.\n";
  } else if (option.detector_type == "FAST") {
    // Parameters:
    // int threshold=10
    // bool nonmaxSuppression=true
    // int type=FastFeatureDetector::TYPE_9_16
    // TODO: Adjust parameters.
    detector_ = cv::FastFeatureDetector::create(30);
    std::cout << "Created FAST Detector.\n";
  } else if (option.detector_type == "SURF") {
    detector_ = cv::xfeatures2d::SURF::create();
    std::cout << "Created SURF Detector.\n";
  } else {
    return;
  }
  if (detector_ == NULL) {
    std::cerr << "Error: Unable to create detector.\n";
    return;
  }

  // If use different detector and descriptor, must explicitly specify.
  if (option.method == FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR &&
      option.detector_type == option.descriptor_type) {
    std::cerr << "Error: Same detector for detector and descriptor.\n";
    return;
  }

  if (option.method == FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR ||
      option.detector_type != option.descriptor_type) {
    detector_type_ = DETECTORDESCRIPTOR;
    if (option.descriptor_type == "DAISY") {
      descriptor_ = cv::xfeatures2d::DAISY::create();
      std::cout << "Created DAISY Descriptor.\n";
    } else if (option.descriptor_type == "ORB") {
      // TODO: Add argument
      descriptor_ = cv::ORB::create();
      std::cout << "Created ORB Descriptor.\n";
    } else if (option.descriptor_type == "FREAK") {
      descriptor_ = cv::xfeatures2d::FREAK::create();
      std::cout << "Created FREAK Descriptor.\n";
    } else if (option.descriptor_type == "SURF") {
      descriptor_ = cv::xfeatures2d::SURF::create();
      std::cout << "Created SURF Descriptor.\n";
    } else {
      return;
    }
    if (descriptor_ == NULL) {
      std::cerr << "Error: Unable to create descriptor.\n";
      return;
    }
  }
  if (matcher == NULL) {
    std::cerr << "Error: No matcher provided to tracker.\n";
    return;
  }
  matcher_ = std::move(matcher);

  FeatureMatcherOptions long_term_matcher_option;
  long_term_matcher_option.method = FeatureMatcherOptions::OCV;

  std::cout << "Creating long term matcher.\n";
  long_term_matcher_ =
      FeatureMatcher::CreateFeatureMatcher(long_term_matcher_option);
}

bool FeatureTrackerOCV::ComputeFrame(ImageFrame &frame) {
  if (!frame.feature_computed()) {
    FeatureTracker::ComputeFeatures(frame);
  }
  return true;
}

bool FeatureTrackerOCV::TrackFrame(ImageFrame &prev_frame,
                                   ImageFrame &new_frame,
                                   std::vector<cv::DMatch> &matches) {
  if (!matcher_) {
    std::cerr << "Error: FeatureMatcher not set up.\n";
    return false;
  }
  if (!prev_frame.feature_computed()) {
    FeatureTracker::ComputeFeatures(prev_frame);
  }
  if (!new_frame.feature_computed()) {
    FeatureTracker::ComputeFeatures(new_frame);
  }

  Timer timer;
  timer.Start();

  if (!MatchFrame(prev_frame, new_frame, matches)) return false;
  // if (!matcher_->Match(prev_frame, new_frame, matches)) return false;

  timer.Stop();
  std::cout << "Matching used " << timer.GetInMs() << "ms.\n";

  return true;
}

bool FeatureTrackerOCV::TrackFrame(ImageFrame &prev_frame,
                                   ImageFrame &new_frame,
                                   const CameraModel *camera_model,
                                   std::vector<cv::DMatch> &matches) {
  if (!matcher_) {
    std::cerr << "Error: FeatureMatcher not set up.\n";
    return false;
  }
  if (!prev_frame.feature_computed()) {
    FeatureTracker::ComputeFeatures(prev_frame, camera_model);
  }
  if (!new_frame.feature_computed()) {
    FeatureTracker::ComputeFeatures(new_frame, camera_model);
  }

  Timer timer;
  timer.Start();

  if (!MatchFrame(prev_frame, new_frame, matches)) return false;
  // if (!matcher_->Match(prev_frame, new_frame, matches)) return false;

  timer.Stop();
  std::cout << "Matching used " << timer.GetInMs() << "ms.\n";

  return true;
}

bool FeatureTrackerOCV::MatchFrame(const ImageFrame &prev_frame,
                                   const ImageFrame &new_frame,
                                   std::vector<cv::DMatch> &matches) {
  // Must already computed features.
  if (!prev_frame.feature_computed() || !new_frame.feature_computed()) {
    return false;
  }

  if (matcher_->Match(prev_frame, new_frame, matches) && matches.size() > 30)
    return true;

  // Otherwise try long term matcher.
  std::cout << "Only " << matches.size()
            << " matches detected, trying long term tracker...\n";

  if (!long_term_matcher_) {
    std::cerr << "Error: Long term FeatureMatcher not set up.\n";
    return false;
  }
  if (!long_term_matcher_->Match(prev_frame, new_frame, matches)) return false;

  std::cout << "Long term tracker found " << matches.size() << " matches.";
  return true;
}

bool FeatureTrackerOCV::DetectFeatures(const ImageFrame &frame,
                                       std::vector<cv::KeyPoint> &kp,
                                       const cv::Mat &mask) {
  detector_->detect(frame.GetImage(), kp, mask);
  return true;
}

bool FeatureTrackerOCV::ComputeDescriptors(const ImageFrame &frame,
                                           std::vector<cv::KeyPoint> &kp,
                                           cv::Mat &desc) {
  if (detector_type_ == DETECTORONLY) {
    detector_->compute(frame.GetImage(), kp, desc);
    // detector_->detectAndCompute(frame.GetImage(), cv::noArray(), kp, desc);
  } else if (detector_type_ == DETECTORDESCRIPTOR) {
    // detector_->detect(frame.GetImage(), kp);
    cv::Mat gray_image;
    cv::cvtColor(frame.GetImage(), gray_image, cv::COLOR_BGR2GRAY);
    descriptor_->compute(gray_image, kp, desc);
    // descriptor_->compute(frame.GetImage(), kp, desc);
  }
  return true;
}

}  // vio
