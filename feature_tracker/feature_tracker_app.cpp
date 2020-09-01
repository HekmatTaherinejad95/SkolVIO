#include <iostream>
#include <string>
#include <unordered_map>

#include "feature_tracker.hpp"
#include "util.hpp"

using namespace std;
using namespace cv;
using namespace vio;

class Options {
 public:
  Options() : draw_match(false) {}
  string path;
  string format;
  string detector;
  string descriptor;
  string matcher_type;
  bool draw_match;
};

int TestFramesInFolder(Options option) {
  vector<string> images;
  if (!GetImageNamesInFolder(option.path, option.format, images)) return -1;
  if (images.size() < 2) {
    cout << "Error: Find only " << images.size() << " images.\n";
    return -1;
  }
  cout << "Testing with " << images.size() << " images.\n";

  // Create tracker
  FeatureMatcherOptions matcher_options;
  if (option.matcher_type.size()) {
    if (option.matcher_type == "GRID") {
      matcher_options.method = vio::FeatureMatcherOptions::GRID_SEARCH;
    } else if (option.matcher_type == "OCV") {
      matcher_options.method = vio::FeatureMatcherOptions::OCV;
    } else {
      cerr << "Error : Unknown matcher type " << option.matcher_type
           << std::endl;
      return -1;
    }
  }
  FeatureTrackerOptions tracker_options;
  if (option.detector.size()) tracker_options.detector_type = option.detector;
  if (option.descriptor.size())
    tracker_options.descriptor_type = option.descriptor;
  tracker_options.max_num_feature = 2000;

  // Change distance function for binary descriptors.
  if (tracker_options.descriptor_type == "ORB" ||
      tracker_options.descriptor_type == "FREAK")
    matcher_options.desc_dist_type = FeatureMatcherOptions::HAMMING;
  if (tracker_options.descriptor_type == "DAISY" ||
      tracker_options.descriptor_type == "SIFT" ||
      tracker_options.descriptor_type == "SURF")
    matcher_options.desc_dist_type = FeatureMatcherOptions::NORM_L2;

  std::unique_ptr<FeatureMatcher> matcher =
      FeatureMatcher::CreateFeatureMatcher(matcher_options);
  std::unique_ptr<FeatureTracker> tracker =
      FeatureTracker::CreateFeatureTracker(tracker_options, std::move(matcher));

  if (tracker == NULL) return -1;

  // Load camera information.
  const std::string config_file_name = "calibration.yaml";
  const std::string full_path_to_config = option.path + '/' + config_file_name;
  cv::FileStorage config_file;
  vio::CameraModelPtr camera_ptr = nullptr;
  config_file.open(full_path_to_config, cv::FileStorage::READ);
  if (!config_file.isOpened()) {
    std::cout << "Couldn't open config file " << full_path_to_config
              << ". Skipped\n";
  } else {
    camera_ptr = vio::CreateCameraModelFromConfig(config_file["CameraModel"]);
    if (camera_ptr == nullptr) {
      std::cerr << "Error: Couldn't create camera model.\n";
      return -1;
    }
  }

  // TODO: Add track length count.

  // Visualize tracking
  cv::namedWindow("result", cv::WINDOW_AUTOSIZE);

  // Track first frame.
  cv::Mat image0 = cv::imread(images[0]);
  std::unique_ptr<vio::ImageFrame> frame_pre(new vio::ImageFrame(image0));

  for (int i = 1; i < images.size(); ++i) {
    cv::Mat image = cv::imread(images[i]);
    std::unique_ptr<vio::ImageFrame> frame_cur(new vio::ImageFrame(image));
    std::vector<cv::DMatch> matches;
    tracker->TrackFrame(*frame_pre, *frame_cur, camera_ptr.get(), matches);

    std::cout << "Feature number in new frame " << frame_cur->keypoints().size()
              << std::endl;
    std::cout << "Found match " << matches.size() << std::endl;

    cv::Mat output_img;

    // Two ways to visualize the results.
    if (option.draw_match) {
      drawMatches(frame_pre->GetImage(), frame_pre->keypoints(),
                  frame_cur->GetImage(), frame_cur->keypoints(), matches,
                  output_img, cv::Scalar(255, 0, 0), cv::Scalar(5, 255, 0));
    } else {
      output_img = frame_cur->GetImage().clone();
      int thickness = 2;
      for (int i = 0; i < matches.size(); ++i) {
        line(output_img, frame_cur->keypoints()[matches[i].trainIdx].pt,
             frame_pre->keypoints()[matches[i].queryIdx].pt,
             cv::Scalar(255, 0, 0), thickness);
      }
      // Draw all features.
      for (auto &kp : frame_cur->keypoints()) {
        circle(output_img, kp.pt, 1, cv::Scalar(0, 0, 255));
      }
    }

    cv::imshow("result", output_img);
    cv::waitKey(0);
    // Store feature tracks.
    // std::unique_ptr<Keyframe> keyframe(new Keyframe(std::move(frame_pre)));

    frame_pre = std::move(frame_cur);
  }
  // TODO: Add test for this.
  // Analyze feature tracks.
  /*
  const std::vector<std::unordered_map<int, int> > &uninited_landmarks =
      vio_map.uninited_landmarks();
  unordered_map<int, int> feature_length_count;
  for (const auto &uninited : uninited_landmarks) {
    int len = uninited.size();
    auto len_ptr = feature_length_count.find(len);
    if (len_ptr == feature_length_count.end())
      feature_length_count[len] = 1;
    else
      feature_length_count[len]++;
  }
  cout << "Stats of feature tracks:\n";
  for (auto feat_len : feature_length_count) {
    cout << "Length " << feat_len.first << " : " << feat_len.second
         << std::endl;
  }
  */
  return 0;
}

int main(int argc, char **argv) {
  Options option;
  for (int i = 0; i < argc; ++i) {
    if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--path")) {
      option.path = argv[++i];
    } else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--format")) {
      option.format = argv[++i];
    } else if (!strcmp(argv[i], "--detector")) {
      option.detector = argv[++i];
    } else if (!strcmp(argv[i], "--descriptor")) {
      option.descriptor = argv[++i];
    } else if (!strcmp(argv[i], "--matcher_type")) {
      option.matcher_type = argv[++i];
    } else if (!strcmp(argv[i], "--draw_match")) {
      option.draw_match = true;
    }
  }

  if (option.format.size() && option.path.size())
    return TestFramesInFolder(option);

  cout << "Error. Unknown arguments.\n";
  cout << "Usage: \n";
  cout << "       test\n";
  cout << "            -p, --path full_path \n";
  cout << "            -f, --format image format, e.g png, jpg\n";
  cout << "            --detector, type of detector, e.g. FAST, ORB\n";
  cout << "            --descriptor, type of descriptor, e.g. ORB, DAISY\n";
  cout << "            --matcher_type, type of feature matcher, e.g. GRID, "
          "OCV\n";
  cout << "Exampe: \n";
  cout << "./feature_tracker_app -p ~/Project/vio/data/desk_subset/ -f jpg\n";

  return 0;
}
