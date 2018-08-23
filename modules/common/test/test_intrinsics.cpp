#include "test.h"

#include <v4r/common/intrinsics.h>

TEST(Intrinsics, getCameraMatrix) {
  v4r::Intrinsics intr{525, 525, 320, 240, 640, 480};
  auto m = intr.getCameraMatrix();
  ASSERT_EQ(m.type(), CV_32FC1);
  ASSERT_EQ(m.size(), cv::Size(3, 3));
  EXPECT_EQ(m.at<float>(0, 0), intr.fx);
  EXPECT_EQ(m.at<float>(0, 1), 0);
  EXPECT_EQ(m.at<float>(0, 2), intr.cx);
  EXPECT_EQ(m.at<float>(1, 0), 0);
  EXPECT_EQ(m.at<float>(1, 1), intr.fy);
  EXPECT_EQ(m.at<float>(1, 2), intr.cy);
  EXPECT_EQ(m.at<float>(2, 0), 0);
  EXPECT_EQ(m.at<float>(2, 1), 0);
  EXPECT_EQ(m.at<float>(2, 2), 1);
}

TEST(Intrinsics, fromCameraMatrixAndResolution) {
  // Float matrix
  {
    cv::Mat m = (cv::Mat_<float>(3, 3) << 505, 0, 320, 0, 515, 240, 0, 0, 1);
    auto intr = v4r::Intrinsics::fromCameraMatrixAndResolution(m, 640, 480);
    EXPECT_EQ(intr.fx, 505);
    EXPECT_EQ(intr.fy, 515);
    EXPECT_EQ(intr.cx, 320);
    EXPECT_EQ(intr.cy, 240);
    EXPECT_EQ(intr.w, 640);
    EXPECT_EQ(intr.h, 480);
  }
  // Double matrix
  {
    cv::Mat m = (cv::Mat_<double>(3, 3) << 505, 0, 320, 0, 515, 240, 0, 0, 1);
    auto intr = v4r::Intrinsics::fromCameraMatrixAndResolution(m, 640, 480);
    EXPECT_EQ(intr.fx, 505);
    EXPECT_EQ(intr.fy, 515);
    EXPECT_EQ(intr.cx, 320);
    EXPECT_EQ(intr.cy, 240);
    EXPECT_EQ(intr.w, 640);
    EXPECT_EQ(intr.h, 480);
  }
}

TEST(Intrinsics, fromCameraMatrixAndResolutionInvalid) {
  cv::Mat_<float> m1(3, 2);
  ASSERT_THROW(v4r::Intrinsics::fromCameraMatrixAndResolution(m1, 640, 480), std::runtime_error);
  cv::Mat_<double> m2(2, 3);
  ASSERT_THROW(v4r::Intrinsics::fromCameraMatrixAndResolution(m2, 640, 480), std::runtime_error);
  cv::Mat_<int> m3(3, 3);
  ASSERT_THROW(v4r::Intrinsics::fromCameraMatrixAndResolution(m3, 640, 480), std::runtime_error);
}

TEST(Intrinsics, loadFromNativeV4RFormat) {
  auto intr = v4r::Intrinsics::load(getTestFilename("common/intrinsics1.txt"));
  EXPECT_EQ(intr.fx, 500);
  EXPECT_EQ(intr.fy, 505);
  EXPECT_EQ(intr.cx, 310);
  EXPECT_EQ(intr.cy, 245);
  EXPECT_EQ(intr.w, 640);
  EXPECT_EQ(intr.h, 480);
}
