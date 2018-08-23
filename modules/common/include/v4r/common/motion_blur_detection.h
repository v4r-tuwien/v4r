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

#ifndef MOTION_BLUR_DETECTION_H
#define MOTION_BLUR_DETECTION_H

#include <v4r/core/macros.h>
#include <memory>
#include <opencv2/core/core.hpp>

namespace v4r {

enum class MotionBlurDetectionType { VL, VML };

class V4R_EXPORTS MotionBlurDetection {
  /** @brief Motion Blur Detection.
   *         Based on Variance of Laplacian (VL) and Variance of Modified Laplacian (VML).
   *
   *  Estimates the motion blur of an image using two different methods (VL, VML). The higher the variance, the sharper
   * the image. Sharp images usually have variances >200.
   */

 public:
  MotionBlurDetection(v4r::MotionBlurDetectionType method) : img_(NULL), method_(method) {}
  MotionBlurDetection(std::shared_ptr<const std::vector<cv::Mat>> im_ptr, v4r::MotionBlurDetectionType method)
  : img_(im_ptr), method_(method) {}

  /** @brief Setup Image Data.
   *
   */
  void setImgVector(std::shared_ptr<const std::vector<cv::Mat>> im_ptr);

  /** @brief Setup detection method. (VL/VML)
   *
   */
  void setDetectionMethod(v4r::MotionBlurDetectionType meth_);

  /** @brief Calculate the motion blur for the provided images.
   *
   * Images have to be provided before the method is called using the constructor or the method
   * MotionBlurDetection::setImgVector.
   *
   */
  void calculateMotionBlurForImgVector(void);

  /** @brief Get measures for a single image.
   *
   */
  double getMotionBlurForImg(const cv::Mat& img) const;

  /** @brief Get measures for provided data.
   * MotionBlurDetection::calculateMotionBlurForImgVector has to be called first.
   *
   */
  const std::vector<double>& getMotionBlurMeasures() const;

 private:
  double calculateVML(const cv::Mat& img) const;
  double calculateVL(const cv::Mat& img) const;

  std::shared_ptr<const std::vector<cv::Mat>> img_;
  std::vector<double> motion_blur_;
  v4r::MotionBlurDetectionType method_;
};
}  // namespace v4r

#endif
