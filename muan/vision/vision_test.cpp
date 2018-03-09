#include "muan/vision/vision.h"
#include "gtest/gtest.h"

TEST(Vision, IgnoresNoise) {
  // Generate 400x400 image, mean 100, stddev 10
  cv::Mat gaussian_noise(400, 400, CV_8UC1);
  cv::randn(gaussian_noise, 100, 10);
  cv::cvtColor(gaussian_noise, gaussian_noise, CV_GRAY2BGR);

  muan::vision::VisionConstants k{1,     1,
                                  0,     0,   // Camera values can be anything
                                  0.001, 1};  // Ignore anything less than .1%

  // Colors are 110-255. About 16% of the noise is within that range.
  muan::vision::VisionThresholds range;
  range.set_a_low(110);
  range.set_b_low(110);
  range.set_c_low(110);
  range.set_a_high(255);
  range.set_b_high(255);
  range.set_c_high(255);
  range.set_space(muan::vision::VisionThresholds::Rgb);

  // No targets should be detected
  muan::vision::Vision vision{range, k};
  ASSERT_EQ(vision.Update(gaussian_noise, gaussian_noise).size(), 0);
}

TEST(Vision, DetectsTargets) {
  muan::vision::VisionThresholds range;
  range.set_a_low(128);
  range.set_b_low(128);
  range.set_c_low(128);
  range.set_a_high(255);
  range.set_b_high(255);
  range.set_c_high(170);
  range.set_space(muan::vision::VisionThresholds::Rgb);

  cv::Mat image(100, 600, CV_8UC3, cv::Scalar(0, 0, 0));
  // Things that aren't targets
  cv::rectangle(image, cv::Point(0, 0), cv::Point(50, 50),
                cv::Scalar(255, 255, 255), -1);
  cv::rectangle(image, cv::Point(100, 0), cv::Point(150, 50),
                cv::Scalar(0, 0, 0), -1);
  cv::rectangle(image, cv::Point(200, 0), cv::Point(250, 50),
                cv::Scalar(127, 160, 160), -1);
  // Things that are targets
  cv::rectangle(image, cv::Point(300, 0), cv::Point(350, 50),
                cv::Scalar(128, 128, 128), -1);
  cv::rectangle(image, cv::Point(400, 0), cv::Point(450, 50),
                cv::Scalar(170, 255, 255), -1);
  cv::rectangle(image, cv::Point(500, 0), cv::Point(550, 50),
                cv::Scalar(160, 200, 200), -1);

  muan::vision::VisionConstants k{1, 1, 0, 0, 0, 1};
  muan::vision::Vision vision{range, k};
  auto targets = vision.Update(image, image);

  ASSERT_EQ(targets.size(), 3);
  for (auto target : targets) {
    // All targets are on the right
    ASSERT_GT(target.x, 0);
    // x-values are (100*n+25)/600
    ASSERT_NEAR((int)(target.x * 600) % 100, 25, 1);
    // Error can be 1 pixel plus floating point error
    ASSERT_NEAR(target.y, 0.25, 0.011);
    ASSERT_NEAR(target.width, 1. / 12., 1. / 599.);
    // Rectangle has fullness 1
    ASSERT_NEAR(target.fullness, 1, 0.01);
  }
}

TEST(Vision, CalculatesPosition) {
  // FovX=1, FovY=0, CameraAngleX=0.5, CameraAngleY=atan(1/2)
  muan::vision::VisionConstants k{1, 0, 0.5, std::atan(1. / 2.), 0, 1};
  muan::vision::Vision vision{muan::vision::VisionThresholds(), k};

  ASSERT_NEAR(vision.CalculateAngle(-0.5), 0, 0.001);
  ASSERT_NEAR(vision.CalculateAngle(0), 0.5, 0.001);
  ASSERT_NEAR(vision.CalculateAngle(0.1), 0.6, 0.001);

  ASSERT_NEAR(vision.CalculateDistance(0, 1), 2, 0.001);
  ASSERT_NEAR(vision.CalculateDistance(1, 1.5), 3, 0.001);
  ASSERT_NEAR(vision.CalculateDistance(0, 2.1), 4.2, 0.001);

  // FovY=1, CameraAngleY=0
  k = muan::vision::VisionConstants{0, 1, 0, 0, 0, 1};
  vision.set_constants(k);

  ASSERT_NEAR(vision.CalculateDistance(std::atan(1. / 2.), 1.5), 3, 0.001);
  ASSERT_NEAR(vision.CalculateDistance(std::atan(1. / 3.), 1), 3, 0.001);
  ASSERT_NEAR(vision.CalculateDistance(std::atan(2. / 5.), 2), 5, 0.001);
}
