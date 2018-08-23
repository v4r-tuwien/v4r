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
 * @file zbuffering.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2012
 * @brief
 *
 */
#pragma once

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <v4r/common/intrinsics.h>
#include <v4r/core/macros.h>
#include <boost/dynamic_bitset.hpp>

namespace v4r {

struct V4R_EXPORTS ZBufferingParameter {
  bool do_smoothing_ = true;  ///< tries to fill holes by dilating points over neighboring pixel
  bool do_noise_filtering_ = false;
  float inlier_threshold_ = 0.01f;
  size_t smoothing_radius_ = 1;
  bool use_normals_ = false;     ///< if true, rejects points that do not point towards view point.
  bool extract_indices_ = true;  ///< if true, computes the indices of the
  ///< original input clouds that are associated
  ///< to the rendered point cloud
};

/**
 * \brief Class to reason about occlusions
 * \author Thomas Faeulhammer, Aitor Aldoma
 */
template <typename PointT>
class V4R_EXPORTS ZBuffering {
 private:
  ZBufferingParameter param_;
  std::vector<int> kept_indices_;
  Intrinsics cam_;             ///< camera parameters
  Eigen::MatrixXi index_map_;  ///< saves for each pixel which indices of the
  ///< input cloud it represents. Non-occupied
  /// pixels are labelled with index -1.
  typename pcl::PointCloud<PointT>::Ptr rendered_view_;
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;  ///< surface normals for input cloud

  /**
   * @brief depthBuffering does depth buffering on input cloud
   * @param cloud input point cloud
   * @param subsample subsampling step size n. If greater 1, will only use every
   * n-th point for rendering
   */
  void depthBuffering(const typename pcl::PointCloud<PointT> &cloud, size_t subsample);

  /**
   * @brief smoothing of rendered cloud to avoid looking through "holes"
   */
  void doSmoothing();

  /**
   * @brief smoothing of rendered cloud to avoid accidentally rendering noisy points
   */
  void doNoiseFiltering();

 public:
  ZBuffering(const Intrinsics &cam, const ZBufferingParameter &p = ZBufferingParameter()) : param_(p), cam_(cam) {}

  /**
   * @brief sets the camera intrinsic parameters
   * @param cam RGB camera intrinsic parameters
   */
  void setCameraIntrinsics(const Intrinsics &cam) {
    cam_ = cam;
  }

  /**
   * @brief setCloudNormals sets the surface normals of the input cloud
   * @param normals surface normals to each point (must have same size as input
   * cloud)
   */
  void setCloudNormals(const pcl::PointCloud<pcl::Normal>::ConstPtr &normals) {
    cloud_normals_ = normals;
  }

  /**
   * @brief renderPointCloud renders a point cloud using the given camera
   * parameters
   * @param cloud input point cloud
   * @param rendered_view[out] rendered point cloud
   * @param subsample subsampling step size n. If greater 1, will only use every
   * n-th point for rendering
   */
  void renderPointCloud(const typename pcl::PointCloud<PointT> &cloud, typename pcl::PointCloud<PointT> &rendered_view,
                        size_t subsample = 1);

  std::vector<int> getKeptIndices() const {
    return kept_indices_;
  }

  /**
   * @brief getIndexMap
   * @return each pixel indicates which point of the input cloud it represents.
   * Non-occupied pixels are labelled with index -1.
   */
  Eigen::MatrixXi getIndexMap() const {
    return index_map_;
  }

  typedef std::shared_ptr<ZBuffering<PointT>> Ptr;
  typedef std::shared_ptr<const ZBuffering<PointT>> ConstPtr;
};

}  // namespace v4r
