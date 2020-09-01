#include "feature_tracker.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

namespace vio {

class FeatureTrackerOCV : public FeatureTracker {
 public:
  enum DetectorType { UNKNOWN = 0, DETECTORONLY, DETECTORDESCRIPTOR };

  FeatureTrackerOCV(FeatureTrackerOptions options,
                    std::unique_ptr<FeatureMatcher> matcher);
  FeatureTrackerOCV() = delete;

  virtual bool ComputeFrame(ImageFrame &frame);

  virtual bool TrackFrame(ImageFrame &prev_frame, ImageFrame &output_frame,
                          const CameraModel *camera_model,
                          std::vector<cv::DMatch> &matches) override;
  virtual bool TrackFrame(ImageFrame &prev_frame, ImageFrame &output_frame,
                          std::vector<cv::DMatch> &matches) override;
  // Use match approach when lost tracking.
  virtual bool MatchFrame(const ImageFrame &prev_frame,
                          const ImageFrame &output_frame,
                          std::vector<cv::DMatch> &matches) override;

 protected:
  bool DetectFeatures(const ImageFrame &frame, std::vector<cv::KeyPoint> &kp,
                      const cv::Mat &mask) override;
  bool ComputeDescriptors(const ImageFrame &frame,
                          std::vector<cv::KeyPoint> &kp,
                          cv::Mat &desc) override;

  DetectorType detector_type_;
  int max_feature_per_frame_;
  cv::Ptr<cv::Feature2D> detector_;
  cv::Ptr<cv::DescriptorExtractor> descriptor_;
};

}  // vio
