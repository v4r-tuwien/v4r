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
 * @file normal_estimator.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */
#pragma once

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/core/macros.h>

namespace v4r {

enum class NormalEstimatorType { PCL_DEFAULT, PCL_INTEGRAL_NORMAL, Z_ADAPTIVE };
V4R_EXPORTS std::istream& operator>>(std::istream& in, NormalEstimatorType& style);
V4R_EXPORTS std::ostream& operator<<(std::ostream& out, const NormalEstimatorType& style);

template <typename PointT>
class V4R_EXPORTS NormalEstimator {
 protected:
  typename pcl::PointCloud<PointT>::ConstPtr input_;  ///< input cloud
  pcl::PointCloud<pcl::Normal>::Ptr normal_;          ///< computed surface normals for input cloud
  std::vector<int>
      indices_;  ///< indices of the segmented object (extracted keypoints outside of this will be neglected)

 public:
  virtual ~NormalEstimator() {}

  /**
   * @brief setInputCloud
   * @param input input cloud
   */
  void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& input) {
    input_ = input;
  }

  /**
   * @brief setIndices
   * @param indices indices of the segmented object (extracted keypoints outside of this will be neglected)
   */
  void setIndices(const std::vector<int>& indices) {
    indices_ = indices;
  }

  /**
   * @brief getNormalEstimatorType
   * @return unique type id of normal estimator(as stated in keypoint/types.h)
   */
  virtual NormalEstimatorType getNormalEstimatorType() const = 0;

  /**
   * @brief compute
   */
  virtual pcl::PointCloud<pcl::Normal>::Ptr compute() = 0;

  typedef std::shared_ptr<NormalEstimator<PointT>> Ptr;
  typedef std::shared_ptr<NormalEstimator<PointT> const> ConstPtr;
};
}  // namespace v4r
