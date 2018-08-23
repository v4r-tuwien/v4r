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
 * @file local_estimator.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2016
 * @brief Local feature descriptor
 *
 */

#pragma once

#include <v4r/common/normal_estimator.h>
#include <v4r/core/macros.h>
#include <opencv2/core/core.hpp>
#include <vector>

namespace v4r {

/**
 * Class for local feature estimation
 * @tparam PointT
 * @author Thomas Faeulhammer
 */
template <typename PointT>
class V4R_EXPORTS LocalEstimator {
 protected:
  typename pcl::PointCloud<PointT>::ConstPtr cloud_;  ///< input cloud
  pcl::PointCloud<pcl::Normal>::ConstPtr normals_;    ///< input normals
  std::vector<int> keypoint_indices_;                 ///< extracted keypoint indices

  std::vector<int> indices_;
  std::string descr_name_;
  size_t descr_type_;
  size_t descr_dims_;
  float max_distance_;  ///< max distance in meters for a keypoint to be valid

 public:
  LocalEstimator() : max_distance_(std::numeric_limits<float>::max()) {}

  virtual ~LocalEstimator() {}

  /**
   * query unique feature type identifier
   * @return unique feature type identifier
   */
  size_t getFeatureType() const {
    return descr_type_;
  }

  /**
   * indicates if feature estimator needs surface normals
   * @return true if surfance normals are required
   */
  virtual bool needNormals() const = 0;

  /**
   * indicates if features estimator intrinsically includes a keypoint detector
   * @return true if keypoint detector is included in feature estimator
   */
  virtual bool detectsKeypoints() const = 0;

  /**
   * @brief set indices of the object (segmented cluster). Points not within this indices will be ignored.
   * @param indices
   */
  void setIndices(const std::vector<int> &indices) {
    indices_ = indices;
  }

  /**
   * @brief sets the normals point cloud
   * @param normals
   */
  void setNormals(const pcl::PointCloud<pcl::Normal>::ConstPtr &normals) {
    normals_ = normals;
  }

  /**
   * @brief sets the input point cloud
   * @param normals
   */
  void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud) {
    cloud_ = cloud;
  }

  /**
   * query keypoint indices with respect to input cloud
   * @return indices of the input cloud that indicate the keypoints
   */
  const std::vector<int> &getKeypointIndices() const {
    return keypoint_indices_;
  }

  /**
   * query the name of the feature descriptor
   * @return name of feature descriptor
   */
  const std::string &getFeatureDescriptorName() const {
    return descr_name_;
  }

  /**
   * query the dimension of the feature descriptor
   * @return the number of feature dimensions
   */
  size_t getFeatureDimensions() const {
    return descr_dims_;
  }

  /**
   * @brief getUniqueId in case several local feature estimators of the same type are used (e.g. shot with different
   * support radii), we need to find unique ids
   * @return a unique identity for the local estimator that takes into account its parameters.
   */
  virtual std::string getUniqueId() const {
    return "";
  }

  /**
   * @brief setMaxDistance sets the maximum distance in meter for a keypoint to be valid
   * @param max_distance maximal distance in meters
   */
  void setMaxDistance(float max_distance) {
    max_distance_ = max_distance;
  }

  /**
   * @brief compute features from given input cloud
   * @param signatures
   */
  virtual void compute(cv::Mat &signatures) = 0;

  typedef std::shared_ptr<LocalEstimator<PointT>> Ptr;
  typedef std::shared_ptr<LocalEstimator<PointT> const> ConstPtr;
};
}  // namespace v4r
