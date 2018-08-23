/****************************************************************************
**
** Copyright (C) 2018 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact
** office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file motion_blur_detection.cpp
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */

#include <opencv2/imgproc/imgproc.hpp>

#include <v4r/common/motion_blur_detection.h>
namespace v4r {
void MotionBlurDetection::calculateMotionBlurForImgVector() {
  if (img_ != nullptr) {
    motion_blur_.clear();
    for (size_t i = 0; i < img_->size(); ++i) {
      double bluriness = getMotionBlurForImg((*img_)[i]);
      motion_blur_.push_back(bluriness);
      // add absolute values
    }
  }
}

void MotionBlurDetection::setImgVector(std::shared_ptr<const std::vector<cv::Mat>> im_ptr) {
  img_ = im_ptr;
}

void MotionBlurDetection::setDetectionMethod(MotionBlurDetectionType meth) {
  method_ = meth;
}

const std::vector<double>& MotionBlurDetection::getMotionBlurMeasures() const {
  return motion_blur_;
}

double MotionBlurDetection::getMotionBlurForImg(const cv::Mat& img) const {
  if (method_ == v4r::MotionBlurDetectionType::VML) {
    return calculateVML(img);
  } else  //(method=VL) v4r::MOTION_BLUR_VL
  {
    return calculateVML(img);
  }
}

double MotionBlurDetection::calculateVL(const cv::Mat& img) const {
  cv::Mat grey(img.rows, img.cols, CV_32F);
  cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);

  cv::Mat_<float> kernel(3, 3);
  kernel << 1., 4, 1., 4., -20., 4., 1., 4., 1.;
  kernel *= 1.0 / 6.0;
  cv::Mat result;
  cv::filter2D(grey, result, CV_32F, kernel);

  cv::Scalar mean, stddev;
  cv::meanStdDev(result, mean, stddev);
  return (stddev[0] * stddev[0]);
}

double MotionBlurDetection::calculateVML(const cv::Mat& img) const {
  cv::Mat dx, dy;
  cv::Mat grey(img.rows, img.cols, CV_32F);
  cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);

  cv::Mat_<float> kernel(1, 3);
  kernel << -1, 2, -1;
  cv::filter2D(grey, dx, CV_32F, kernel);

  cv::Mat kernel_t;
  cv::transpose(kernel, kernel_t);
  cv::filter2D(grey, dy, CV_32F, kernel);

  cv::Mat result = cv::abs(dx) + cv::abs(dy);

  cv::Scalar mean, stddev;
  cv::meanStdDev(result, mean, stddev);
  return (stddev[0] * stddev[0]);
}
}  // namespace v4r
