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
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <v4r/config.h>
#include <v4r/features/FeatureDetector.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if CV_VERSION_MAJOR < 3
#include <opencv2/nonfree/features2d.hpp>  // requires OpenCV non-free module
#else
#include <opencv2/xfeatures2d.hpp>
#endif

namespace v4r {

class V4R_EXPORTS FeatureDetector_KD_CVSIFT : public FeatureDetector {
 public:
  struct V4R_EXPORTS Parameter {
    int nfeatures = 0;
    int nOctaveLayers = 4;
    double contrastThreshold = 0.02;
    double edgeThreshold = 5;
    double sigma = 1.6;

    Parameter() {}
  };

 private:
  using FeatureDetector::descr_name_;
  Parameter param;

#if CV_VERSION_MAJOR < 3
  cv::Ptr<cv::SIFT> sift;
#else
  cv::Ptr<cv::Feature2D> sift;
#endif

  cv::Mat_<unsigned char> im_gray_;

 public:
  FeatureDetector_KD_CVSIFT(const Parameter &_p = Parameter());
  ~FeatureDetector_KD_CVSIFT() {}

  void detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors,
                        const cv::Mat &object_mask = cv::Mat()) override final;
  void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
              const cv::Mat &object_mask = cv::Mat()) override final;
  void compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) override final;

  typedef std::shared_ptr<FeatureDetector_KD_CVSIFT> Ptr;
  typedef std::shared_ptr<FeatureDetector_KD_CVSIFT const> ConstPtr;
};

/*************************** INLINE METHODES **************************/

}  // namespace v4r
