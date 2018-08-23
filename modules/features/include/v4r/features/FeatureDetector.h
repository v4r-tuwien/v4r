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
 * @brief Class which extract keypoints and/or compute features from 2D images
 *
 */

#pragma once

#include <v4r/core/macros.h>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace v4r {

class V4R_EXPORTS FeatureDetector {
 public:
  enum class Type {
    K_MSER,
    K_HARRIS,
    KD_CVSURF,
    KD_CVSIFT,
    KD_SIFTGPU,
    D_FREAK,
    KD_ORB,
    KD_FAST_IMGD,
    KD_PSURF,
    KD_MSER_IMGD,
    KD_HARRIS_IMGD,
    KD_PSURF_FREAK,
    KD_PSURF_IMGD,
    KD_FAST_PSURF,
    KD_FAST_SIFTGPU,
    KD_CVSURF_FREAK,
    KD_CVSURF_IMGD,
    KD_FAST_CVSURF,
    KD_SIFTGPU_IMGD,
    KD_AKAZE,
    KD_BRISK,
    KD_SURF
  };

 private:
  Type type;

 protected:
  std::string descr_name_;
  std::vector<int> keypoint_indices_;  ///< extracted keypoint indices

  /**
   * @brief compute keypoint indices for a given set of keypoints extracted from an image. The indices correspond to the
   * pixel values stacked into row-major order
   * @param image input image
   * @param keys keypoints
   */
  void computeKeypointIndices(const cv::Mat &image, const std::vector<cv::KeyPoint> &keys);

 public:
  FeatureDetector(Type _type) : type(_type) {}
  virtual ~FeatureDetector() {}

  /**
   * @brief Extracts keypoints of an image and computes a feature descriptor for each of them
   * @param image Input image from which to extract features
   * @param keys extracted keypoints (points of interest) from the image
   * @param descriptors feature descriptors for each keypoint
   * @param object_mask object mask (pixels of the image for which extracted keypoints are accepted). If empty, all
   * extracted keypoints are valid.
   */
  virtual void detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors,
                                const cv::Mat &object_mask = cv::Mat()) {
    (void)image;
    (void)keys;
    (void)descriptors;
    (void)object_mask;
    std::cout << "[FeatureDetector::detectAndCompute] Not implemented!]" << std::endl;
  };

  /**
   * @brief Extracts keypoints of an image
   * @param image Input image from which to extract features
   * @param keys extracted keypoints (points of interest) from the image
   * @param object_mask object mask (pixels of the image for which extracted keypoints are accepted). If empty, all
   * extracted keypoints are valid.
   */
  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, const cv::Mat &object_mask = cv::Mat()) {
    (void)image;
    (void)keys;
    (void)object_mask;
    std::cout << "[FeatureDetector::detect] Not implemented!]" << std::endl;
  }

  /**
   * @brief Computes a feature descriptor for a given set of keypoints
   * @param image Input image
   * @param keys keypoints for which to compute features (keypoints for which no valid descriptor can be computed, will
   * be removed from the vector)
   * @param descriptors feature descriptors for each keypoint
   */
  virtual void compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
    (void)image;
    (void)keys;
    (void)descriptors;
    std::cout << "[FeatureDetector::compute] Not implemented!]" << std::endl;
  }

  Type getType() const {
    return type;
  }

  /**
   * query keypoint indices with respect to input cloud
   * @return indices of the input cloud that indicate the keypoints
   */
  const std::vector<int> &getKeypointIndices() const {
    return keypoint_indices_;
  }

  const std::string &getDescriptorName() const {
    return descr_name_;
  }

  typedef std::shared_ptr<FeatureDetector> Ptr;
  typedef std::shared_ptr<FeatureDetector const> ConstPtr;
};

V4R_EXPORTS std::istream &operator>>(std::istream &in, FeatureDetector::Type &t);
V4R_EXPORTS std::ostream &operator<<(std::ostream &out, const FeatureDetector::Type &t);

}  // namespace v4r
