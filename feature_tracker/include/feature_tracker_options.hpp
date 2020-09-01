#ifndef VIO_FEATURE_TRACKER_OPTIONS_
#define VIO_FEATURE_TRACKER_OPTIONS_

#include <opencv2/opencv.hpp>

namespace vio {

class FeatureTrackerOptions {
 public:
  FeatureTrackerOptions()
      : method(OCV_BASIC_DETECTOR_EXTRACTOR),
        detector_type("FAST"),
        max_num_feature(2000),
        descriptor_type("FREAK") {}

  enum FeatureTrackerMethod {
    OCV_BASIC_DETECTOR = 0,
    OCV_BASIC_DETECTOR_EXTRACTOR,
  };

  void read(const cv::FileNode &node) {
    method = static_cast<FeatureTrackerMethod>((int)node["Method"]);
    detector_type = (std::string)node["DetectorType"];
    max_num_feature = (int)node["max_num_feature"];
    descriptor_type = (std::string)node["DescriptorType"];
  }

  FeatureTrackerMethod method;

  // Detector
  std::string detector_type;
  int max_num_feature;
  // Descriptor
  std::string descriptor_type;
};

// Following must be defined for the serialization in FileStorage to work
static void read(
    const cv::FileNode &node, FeatureTrackerOptions &x,
    const FeatureTrackerOptions &default_value = FeatureTrackerOptions()) {
  if (node.empty())
    x = default_value;
  else
    x.read(node);
}

}  // vio

#endif
