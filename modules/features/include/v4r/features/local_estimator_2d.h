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
 * @file local_estimator_2d.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2018
 * @brief base class for all feature estimators for 2D images
 *
 */
#pragma once

#include <pcl/point_cloud.h>
#include <v4r/config.h>
#include <v4r/features/FeatureDetector.h>
#include <v4r/features/local_estimator.h>
#include <v4r/features/types.h>
#include <opencv2/core/core.hpp>

namespace v4r {

template <typename PointT>
class V4R_EXPORTS LocalEstimator2D : public LocalEstimator<PointT> {
 protected:
  using LocalEstimator<PointT>::keypoint_indices_;
  using LocalEstimator<PointT>::descr_name_;
  using LocalEstimator<PointT>::cloud_;
  using LocalEstimator<PointT>::indices_;
  using LocalEstimator<PointT>::max_distance_;

  FeatureDetector::Ptr feat_;               ///< feature descriptor (and also keypoint detector if not set explicitly)
  FeatureDetector::Ptr keypoint_detector_;  ///< explicit keypoint detector (optional if feature descriptor has already
                                            ///< one implicitly included)

  /**
   * @brief remove all keypoints and their associated signatures and keypoint indices that fall on a non-valid point
   */
  void removeNanKeypoints(cv::Mat &signatures, std::vector<cv::KeyPoint> &keypoints2d);

  /**
   * @brief set descriptor name. If keypoint detector is different than feature descriptor, it will include both names
   * in the descriptor name. Otherwise just the feature descriptor name
   */
  void setDescriptorName() {
    if (feat_ && keypoint_detector_ && feat_->getDescriptorName() != keypoint_detector_->getDescriptorName()) {
      descr_name_ = keypoint_detector_->getDescriptorName() + "_" + feat_->getDescriptorName();
    } else if (feat_) {
      descr_name_ = feat_->getDescriptorName();
    }
  }

 public:
  /**
   * @brief compute features of an image
   * @param[in] colorImage color image from which to compute features
   * @param[out] keypoints extracted keypoints
   * @param[out] signatures computed signatures to each keypoint
   * @param[in] object_mask optional object mask (region of interest) which indicates to which points to compute
   * keypoints
   */
  virtual void compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &signatures,
                       const cv::Mat &object_mask = cv::Mat());

  void compute(cv::Mat &signatures) override;

  bool detectsKeypoints() const override {
    return true;  /// TODO: implement this again
  }

  bool needNormals() const override {
    return false;
  }

  void setFeatureDetector(const FeatureDetector::Ptr &f) {
    feat_ = f;
    setDescriptorName();
  }

  void setKeypointDetector(const FeatureDetector::Ptr &f) {
    keypoint_detector_ = f;
    setDescriptorName();
  }

  typedef std::shared_ptr<LocalEstimator2D<PointT>> Ptr;
  typedef std::shared_ptr<LocalEstimator2D<PointT> const> ConstPtr;
};
}  // namespace v4r
