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

#ifndef EPMATH_H
#define EPMATH_H

#include <v4r/core/macros.h>

#include "v4r/attention_segmentation/eputils_headers.h"

namespace v4r {

float dotProduct(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);
float dotProduct(const cv::Point3f &v1, const cv::Point3f &v2);

float vectorLength(const Eigen::Vector3f &v);
float vectorLength(const cv::Point3d &v);

V4R_EXPORTS float calculateCosine(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);
V4R_EXPORTS float calculateCosine(const cv::Point3d &v1, const cv::Point3d &v2);

V4R_EXPORTS Eigen::Vector3f normalize(Eigen::Vector3f v);
V4R_EXPORTS cv::Point3d normalize(cv::Point3d v);

Eigen::Vector3f crossProduct(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);
cv::Point3d crossProduct(const cv::Point3d &v1, const cv::Point3d &v2);

Eigen::Vector3f crossProduct(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3);
cv::Point3d crossProduct(const cv::Point3d &p1, const cv::Point3d &p2, const cv::Point3d &p3);

Eigen::Vector3f calculatePlaneNormal(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);
Eigen::Vector3f calculatePlaneNormal(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3);

cv::Point3d calculatePlaneNormal(const cv::Point3d &v1, const cv::Point3d &v2);
cv::Point3d calculatePlaneNormal(const cv::Point3d &p1, const cv::Point3d &p2, const cv::Point3d &p3);

#ifndef NOT_USE_PCL

float dotProduct(const pcl::Normal &v1, const pcl::Normal &v2);
float dotProduct(const pcl::PointXYZ &v1, const pcl::PointXYZ &v2);

float vectorLength(const pcl::Normal &v);
float vectorLength(const pcl::PointXYZ &v);

V4R_EXPORTS float calculateCosine(const pcl::Normal &v1, const pcl::Normal &v2);
V4R_EXPORTS float calculateCosine(const pcl::PointXYZ &v1, const pcl::PointXYZ &v2);

pcl::Normal normalize(pcl::Normal v);
pcl::PointXYZ normalize(pcl::PointXYZ v);

pcl::Normal crossProduct(const pcl::Normal &v1, const pcl::Normal &v2);
pcl::PointXYZ crossProduct(const pcl::PointXYZ &v1, const pcl::PointXYZ &v2);

pcl::Normal crossProduct(const pcl::Normal &p1, const pcl::Normal &p2, const pcl::Normal &p3);
pcl::PointXYZ crossProduct(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3);

V4R_EXPORTS pcl::Normal calculatePlaneNormal(const pcl::Normal &v1, const pcl::Normal &v2);
pcl::Normal calculatePlaneNormal(const pcl::Normal &p1, const pcl::Normal &p2, const pcl::Normal &p3);
pcl::PointXYZ calculatePlaneNormal(const pcl::PointXYZ &v1, const pcl::PointXYZ &v2);
pcl::PointXYZ calculatePlaneNormal(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3);

V4R_EXPORTS void ProjectPointsOnThePlane(
    pcl::ModelCoefficients::ConstPtr coefficients, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_projected, std::vector<float> &distances,
    pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices()), bool normalize = true);

template <class T>
bool computeMean(const typename pcl::PointCloud<T> &cloud, Eigen::Vector3f &mean,
                 std::vector<int> indices = std::vector<int>()) {
  if (indices.size() == 0) {
    indices.reserve(cloud.size());
    for (unsigned int i = 0; i < cloud.size(); ++i) {
      if (std::isnan(cloud.points.at(i).x) || std::isnan(cloud.points.at(i).y) || std::isnan(cloud.points.at(i).z)) {
        continue;
      } else {
        indices.push_back(i);
      }
    }
  }

  if (indices.size() == 0)
    return (false);

  mean.setZero();
  for (unsigned i = 0; i < indices.size(); ++i) {
    int idx = indices.at(i);
    mean[0] += cloud.points.at(idx).x;
    mean[1] += cloud.points.at(idx).y;
    mean[2] += cloud.points.at(idx).z;
  }

  mean /= (float)indices.size();

  return (true);
}

template <class T>
bool computeCovarianceMatrix(const typename pcl::PointCloud<T> &cloud, const Eigen::Vector3f &mean,
                             Eigen::Matrix3f &cov, std::vector<int> indices = std::vector<int>()) {
  if (indices.size() == 0) {
    indices.reserve(cloud.size());
    for (unsigned int i = 0; i < cloud.size(); ++i) {
      if (std::isnan(cloud.points.at(i).x) || std::isnan(cloud.points.at(i).y) || std::isnan(cloud.points.at(i).z)) {
        continue;
      } else {
        indices.push_back(i);
      }
    }
  }

  bool done = false;
  cov.setZero();

  for (unsigned pi = 0; pi < indices.size(); ++pi) {
    float x = cloud.points.at(indices.at(pi)).x - mean[0];
    float y = cloud.points.at(indices.at(pi)).y - mean[1];
    float z = cloud.points.at(indices.at(pi)).z - mean[2];

    cov(0, 0) += x * x;
    cov(0, 1) += x * y;
    cov(0, 2) += x * z;

    cov(1, 0) += y * x;
    cov(1, 1) += y * y;
    cov(1, 2) += y * z;

    cov(2, 0) += z * x;
    cov(2, 1) += z * y;
    cov(2, 2) += z * z;

    done = true;
  }

  return (done);
}

#endif

}  // namespace v4r

#endif
