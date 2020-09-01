#include "feature_tracker.hpp"

#include "../../util/include/timer.hpp"

namespace vio {

std::unique_ptr<FeatureTracker> FeatureTracker::CreateFeatureTracker(
    FeatureTrackerOptions option, std::unique_ptr<FeatureMatcher> matcher) {
  switch (option.method) {
    case FeatureTrackerOptions::OCV_BASIC_DETECTOR:
    case FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR:
      return CreateFeatureTrackerOCV(option, std::move(matcher));
    default:
      return nullptr;
  }
}

void FeatureTracker::ComputeFeatures(ImageFrame &frame) {
  ComputeFeatures(frame, nullptr);
}

// Detect features in the entire image.
void FeatureTracker::ComputeFeatures(ImageFrame &frame,
                                     const CameraModel *camera_model) {
  Timer timer;
  timer.Start();

  if (camera_model) {
    cv::Mat undistorted_image;

    Timer undistort_timer;
    undistort_timer.Start();
    camera_model->UndistortImage(frame.GetImage(), undistorted_image);

    // cv::namedWindow("original", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("undistorted", cv::WINDOW_AUTOSIZE);
    // cv::imshow("original", frame.GetImage());

    frame.ResetImage(undistorted_image);

    undistort_timer.Stop();
    undistort_timer.PrintDurationWithInfo("Undistort image");

    // cv::imshow("undistorted", frame.GetImage());
    // cv::waitKey(0);
  }
  // cv::Mat mask = cv::Mat::zeros(frame.GetImage().size(), CV_8U);
  cv::Mat mask(frame.GetImage().size(), CV_8U);
  mask = cv::Scalar(255);

  // cv::Mat roi(mask, cv::Rect(0, 0, 200, 200));
  // roi = cv::Scalar(255);

  std::vector<cv::KeyPoint> kp;
  DetectFeatures(frame, kp, mask);
  cv::Mat desc;
  ComputeDescriptors(frame, kp, desc);

  /*
  // If apply camera model.
  if (camera_model) {
    for (auto &keypoint : kp) {
      std::cout << "Undistorted (" << keypoint.pt.x << ", " << keypoint.pt.y
                << ") to (";
      Eigen::Vector2d original_kp(keypoint.pt.x, keypoint.pt.y);
      Eigen::Vector2d undistorted_kp;
      camera_model->UndistortPixel(original_kp, undistorted_kp);
      keypoint.pt.x = undistorted_kp[0];
      keypoint.pt.y = undistorted_kp[1];

      std::cout << keypoint.pt.x << ", " << keypoint.pt.y << ")\n";
    }
  }
  */

  frame.set_features(kp, desc);

  timer.Stop();
  std::cout << "Detect and compute used " << timer.GetInMs() << "ms.\n";
}

// TODO: Finish when the odometry is done.
void FeatureTracker::ComputeDistributedFeatures(ImageFrame &frame) {
  // Split the image to |num_bin_col_| x |num_bin_row_| grids.
  const int max_num_feat_per_grid =
      max_num_feature_ / num_bin_col_ / num_bin_row_;

  cv::Mat mask(frame.GetImage().size(), CV_8U);
  cv::Mat roi(mask, cv::Rect(0, 0, 10, 10));
  roi = cv::Scalar(255);
}

}  // vio
