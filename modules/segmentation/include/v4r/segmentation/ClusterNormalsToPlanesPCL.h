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
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <flann/flann.h>
#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/common/plane_model.h>
#include <v4r/core/macros.h>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <set>
#include <vector>

namespace v4r {

class ClusterNormalsToPlanesPCLParameter {
 public:
  double thrAngle;     ///< Threshold of angle for normal clustering
  double inlDist;      ///< Maximum inlier distance
  unsigned minPoints;  ///< Minimum number of points for a plane
  bool least_squares_refinement;
  bool smooth_clustering;
  double thrAngleSmooth;  ///< Threshold of angle for normal clustering
  double inlDistSmooth;   ///< Maximum inlier distance
  unsigned minPointsSmooth;
  int K_;  // k in nearest neighor search when doing smooth clustering in unorganized point clouds
  int normal_computation_method_;  ///< defines the method used for normal computation (only used when point cloud is
                                   /// downsampled / unorganized)

  ClusterNormalsToPlanesPCLParameter(double thrAngleNC = 30, double _inlDist = 0.01, unsigned _minPoints = 9,
                                     bool _least_squares_refinement = true, bool _smooth_clustering = false,
                                     double _thrAngleSmooth = 30, double _inlDistSmooth = 0.02,
                                     unsigned _minPointsSmooth = 3, int K = 5, int normal_computation_method = 2)
  : thrAngle(thrAngleNC), inlDist(_inlDist), minPoints(_minPoints), least_squares_refinement(_least_squares_refinement),
    smooth_clustering(_smooth_clustering), thrAngleSmooth(_thrAngleSmooth), inlDistSmooth(_inlDistSmooth),
    minPointsSmooth(_minPointsSmooth), K_(K), normal_computation_method_(normal_computation_method) {}
};

/**
 * ClusterNormalsToPlanes
 */
template <typename PointT>
class V4R_EXPORTS ClusterNormalsToPlanesPCL {
  /**
   * @brief The Plane class
   */
  class Plane {
   public:
    bool is_plane;
    Eigen::Vector3f pt;
    Eigen::Vector3f normal;
    std::vector<int> indices;

    void clear() {
      pt.setZero();
      normal.setZero();
      indices.clear();
    }

    inline void init(const Eigen::Vector3f &_pt, const Eigen::Vector3f &_n, size_t idx) {
      pt = _pt;
      normal = _n;
      indices.resize(1);
      indices[0] = (int)idx;
    }

    inline void add(const Eigen::Vector3f &_pt, const Eigen::Vector3f &_n,
                    size_t idx) {  // update average normal and point
      double w = indices.size() / double(indices.size() + 1);
      pt = pt * w + _pt / double(indices.size() + 1);  // this should be numerically more stable than the previous
                                                       // version (but more division / computationally expensive)
      normal = normal * w + _n / double(indices.size() + 1);
      indices.push_back((int)idx);
    }

    inline unsigned size() {
      return indices.size();
    }

    Plane(bool _is_plane = false) : is_plane(_is_plane) {}
    ~Plane() {}
    typedef std::shared_ptr<::v4r::ClusterNormalsToPlanesPCL<PointT>::Plane> Ptr;
    typedef std::shared_ptr<::v4r::ClusterNormalsToPlanesPCL<PointT>::Plane const> ConstPtr;
  };

 private:
  typedef flann::L1<float> DistT;
  ClusterNormalsToPlanesPCLParameter param;
  float cos_rad_thr_angle, cos_rad_thr_angle_smooth;

  std::vector<bool> mask_;
  std::vector<int> queue_;

  std::shared_ptr<flann::Index<DistT>> flann_index;  // for unorganized point clouds;

  // cluster normals
  void doClustering(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                    const pcl::PointCloud<pcl::Normal> &normals,
                    std::vector<typename ClusterNormalsToPlanesPCL<PointT>::Plane::Ptr> &planes);
  // cluster normals from point
  void clusterNormals(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                      const pcl::PointCloud<pcl::Normal> &normals, size_t idx, Plane &plane);
  // cluster normals from point for an unorganized pointcloud
  void clusterNormalsUnorganized(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                 const pcl::PointCloud<pcl::Normal> &normals, size_t idx, Plane &plane);
  // do a smooth clustering
  void smoothClustering(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                        const pcl::PointCloud<pcl::Normal> &normals, size_t idx, Plane &plane);
  // adds normals to each point of segmented patches

 public:
  ClusterNormalsToPlanesPCL(const ClusterNormalsToPlanesPCLParameter &_p = ClusterNormalsToPlanesPCLParameter())
  : param(_p) {
    cos_rad_thr_angle = cos(pcl::deg2rad(param.thrAngle));
    cos_rad_thr_angle_smooth = cos(pcl::deg2rad(param.thrAngleSmooth));
  }

  ~ClusterNormalsToPlanesPCL() {}

  /** Compute planes by surface normal grouping **/
  void compute(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const pcl::PointCloud<pcl::Normal> &normals,
               std::vector<PlaneModel<PointT>> &_planes);

  /** Compute a plane starting from a seed point **/
  void compute(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const pcl::PointCloud<pcl::Normal> &normals,
               int x, int y, PlaneModel<PointT> &pm);

  typedef std::shared_ptr<::v4r::ClusterNormalsToPlanesPCL<PointT>> Ptr;
  typedef std::shared_ptr<::v4r::ClusterNormalsToPlanesPCL<PointT> const> ConstPtr;
};
}  // namespace v4r
