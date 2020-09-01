#ifndef VIO_FEATURE_TRACKER_
#define VIO_FEATURE_TRACKER_

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

#include "camera_model.hpp"
#include "feature_matcher.hpp"
#include "feature_tracker_options.hpp"
#include "image_frame.hpp"

namespace vio {

class FeatureTracker {
 public:
  FeatureTracker()
      : num_bin_col_(25), num_bin_row_(25), max_num_feature_(2000) {}
  ~FeatureTracker() {}

  static std::unique_ptr<FeatureTracker> CreateFeatureTracker(
      FeatureTrackerOptions option, std::unique_ptr<FeatureMatcher> matcher);

  static std::unique_ptr<FeatureTracker> CreateFeatureTrackerOCV(
      FeatureTrackerOptions option, std::unique_ptr<FeatureMatcher> matcher);

  virtual bool ComputeFrame(ImageFrame &frame) = 0;

  virtual bool TrackFrame(ImageFrame &prev_frame, ImageFrame &output_frame,
                          const CameraModel *camera_model,
                          std::vector<cv::DMatch> &matches) = 0;
  virtual bool TrackFrame(ImageFrame &prev_frame, ImageFrame &output_frame,
                          std::vector<cv::DMatch> &matches) = 0;
  virtual bool MatchFrame(const ImageFrame &prev_frame,
                          const ImageFrame &output_frame,
                          std::vector<cv::DMatch> &matches) = 0;

 protected:
  virtual bool DetectFeatures(const ImageFrame &frame,
                              std::vector<cv::KeyPoint> &kp,
                              const cv::Mat &mask) = 0;
  virtual bool ComputeDescriptors(const ImageFrame &frame,
                                  std::vector<cv::KeyPoint> &kp,
                                  cv::Mat &desc) = 0;

  // Detect features in the entire image.
  void ComputeFeatures(ImageFrame &frame);
  void ComputeFeatures(ImageFrame &frame, const CameraModel *camera_model);

  void ComputeDistributedFeatures(ImageFrame &frame);

 protected:
  // matcher for tracking
  std::unique_ptr<FeatureMatcher> matcher_;
  // TODO: Make an argument to create tracker.
  std::unique_ptr<FeatureMatcher> long_term_matcher_;

  int num_bin_col_;
  int num_bin_row_;
  int max_num_feature_;
};

typedef std::unique_ptr<FeatureTracker> FeatureTrackerPtr;

}  // vio

#endif
