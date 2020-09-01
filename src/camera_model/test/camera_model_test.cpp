#include "gtest/gtest.h"

#include "camera_model.hpp"

class CameraModelTest : public ::testing::Test {
 protected:
  template <class CameraModel>
  void TestProjectPoint(const CameraModel &camera, const Eigen::Vector3d &point,
                        const Eigen::Vector2d &expected_pixel,
                        bool expected_success) {
    Eigen::Vector2d pixel;
    camera.image_height();
    bool success = camera.ProjectPoint(point, pixel);
    ASSERT_EQ(success, expected_success);
    if (!expected_success) return;
    ASSERT_TRUE(pixel.isApprox(expected_pixel));
  }
};

TEST_F(CameraModelTest, TestPinholeCameraOriginAtPrincipalPoint) {
  vio::PinholeCameraModel<double>::ParamsArray params;
  // Principal point is camera center.
  params << 1.0, 2.0, 0, 0, 0, 0, 0, 0;
  vio::PinholeCameraModel<double> camera(480, 640, params);
  // A point on center ray
  TestProjectPoint(camera, Eigen::Vector3d(0, 0, 10), Eigen::Vector2d(0, 0),
                   true);
  TestProjectPoint(camera, Eigen::Vector3d(10, 5, 10), Eigen::Vector2d(1, 1),
                   true);
}

TEST_F(CameraModelTest, TestPinholeCameraNormalCases) {
  vio::PinholeCameraModel<double>::ParamsArray params;
  params << 1.0, 2.0, 3.0, 4.0, 0, 0, 0, 0;
  vio::PinholeCameraModel<double> camera(480, 640, params);
  TestProjectPoint(camera, Eigen::Vector3d(0, 0, 1), Eigen::Vector2d(3, 4),
                   true);
  TestProjectPoint(camera, Eigen::Vector3d(1, 1, 1), Eigen::Vector2d(4, 6),
                   true);
}

// TODO: For expected false test, don't need to input expected pixel.
TEST_F(CameraModelTest, TestPinholeCameraFailedCases) {
  vio::PinholeCameraModel<double>::ParamsArray params;
  params << 1.0, 2.0, 3.0, 4.0, 0, 0, 0, 0;
  vio::PinholeCameraModel<double> camera(480, 640, params);

  // Behind the camera.
  TestProjectPoint(camera, Eigen::Vector3d(1, 1, -1), Eigen::Vector2d(0, 0),
                   false);
  // Test out of image.
  TestProjectPoint(camera, Eigen::Vector3d(1000, 1000, 1),
                   Eigen::Vector2d(0, 0), false);
  TestProjectPoint(camera, Eigen::Vector3d(-10, 10, 1), Eigen::Vector2d(0, 0),
                   false);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
