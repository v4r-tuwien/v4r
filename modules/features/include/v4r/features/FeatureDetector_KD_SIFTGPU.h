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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <SiftGPU/SiftGPU.h>
#include <v4r/features/FeatureDetector.h>

namespace v4r {

class V4R_EXPORTS FeatureDetector_KD_SIFTGPU : public FeatureDetector {
 public:
  struct V4R_EXPORTS Parameter {
    float distmax = FLT_MAX;      // absolute descriptor distance (e.g. = 0.6)
    float ratiomax = 1.f;         // compare best match with second best (e.g. =0.8)
    int mutual_best_match = 0;    // compare forward/backward matches (1)
    bool computeRootSIFT = true;  // L1 norm and square root => euc dist = hellinger dist
    Parameter() {}
  };

 private:
  using FeatureDetector::descr_name_;
  Parameter param;

  cv::Ptr<SiftGPU> sift;
  cv::Mat_<unsigned char> im_gray_;

  /**
   * @brief TransformToRootSIFT computes the square root of the L1 normalized SIFT vectors. Then the Euclidean distance
   * is equivalent to using the Hellinger kernel to compare the original SIFT vectors
   * @param descriptors feature descriptors
   */
  void transformToRootSIFT(cv::Mat &descriptors) const;

  inline float distance128(float d1[128], float d2[128]);
  inline float sqr(const float &a);

 public:
  FeatureDetector_KD_SIFTGPU(const Parameter &_p = Parameter(), const cv::Ptr<SiftGPU> &_sift = cv::Ptr<SiftGPU>());
  ~FeatureDetector_KD_SIFTGPU();

  void detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors,
                        const cv::Mat &object_mask = cv::Mat()) override final;
  void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
              const cv::Mat &object_mask = cv::Mat()) override final;
  void compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) override final;

  static std::vector<std::pair<int, int>> matchSIFT(const cv::Mat &desc1, const cv::Mat &desc2);

  typedef std::shared_ptr<FeatureDetector_KD_SIFTGPU> Ptr;
  typedef std::shared_ptr<FeatureDetector_KD_SIFTGPU const> ConstPtr;
};

/*************************** INLINE METHODES **************************/

inline float FeatureDetector_KD_SIFTGPU::distance128(float d1[128], float d2[128]) {
  float sqrDist = 0.;

  for (unsigned i = 0; i < 128; i++)
    sqrDist += sqr(d1[i] - d2[i]);

  return sqrt(sqrDist);
}

inline float FeatureDetector_KD_SIFTGPU::sqr(const float &a) {
  return a * a;
}

}  // namespace v4r
