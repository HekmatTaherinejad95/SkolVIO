#ifndef VIO_FEATURE_MATCHER_OCV_
#define VIO_FEATURE_MATCHER_OCV_

#include "feature_matcher.hpp"

#include <vector>

namespace vio {

class FeatureMatcherOCV : public FeatureMatcher {
 public:
  FeatureMatcherOCV(const FeatureMatcherOptions &option);
  ~FeatureMatcherOCV(){};

  virtual bool Match(const ImageFrame &frame0, const ImageFrame &frame1,
                     std::vector<cv::DMatch> &matches);

 private:
  cv::Ptr<cv::DescriptorMatcher> matcher_;
};

}  // vio

#endif
