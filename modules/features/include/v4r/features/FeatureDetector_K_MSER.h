/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
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
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2018
 * @brief OpenCV MSER keypoint extractor implementation
 *
 */

#pragma once

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include "FeatureDetector.h"

namespace v4r {

class V4R_EXPORTS FeatureDetector_K_MSER : public FeatureDetector {
 public:
  struct V4R_EXPORTS Parameter {
    int delta_ = 5;                ///< compares \f$(size_{i}-size_{i-delta})/size_{i-delta}\f$
    int min_area_ = 60;            ///< prune the area which smaller than minArea
    int max_area_ = 14400;         ///< prune the area which bigger than maxArea
    double max_variation_ = 0.25;  ///< prune the area have similar size to its children
    double min_diversity_ = .2;  ///< for color image, trace back to cut off mser with diversity less than min_diversity
    int max_evolution_ = 200;    ///< for color image, the evolution steps
    double area_threshold_ = 1.01;  ///< for color image, the area threshold to cause re-initialize
    double min_margin_ = 0.003;     ///< for color image, ignore too small margin
    int edge_blur_size_ = 5;        ///< for color image, the aperture size for edge blur
    Parameter() {}
  };

 private:
  using FeatureDetector::descr_name_;
  Parameter param;
  cv::Mat_<unsigned char> im_gray_;

  cv::Ptr<cv::MSER> mser_;

 public:
  FeatureDetector_K_MSER(const Parameter &_p = Parameter());

  void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
              const cv::Mat &object_mask = cv::Mat()) override final;

  typedef std::shared_ptr<FeatureDetector_K_MSER> Ptr;
  typedef std::shared_ptr<FeatureDetector_K_MSER const> ConstPtr;
};

/*************************** INLINE METHODES **************************/

}  // namespace v4r
