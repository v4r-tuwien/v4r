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
 * @file BoundaryRelationsMeanCurvature.h
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Class to calculate boundary curvature and standard deviation.
 */

#include "v4r/attention_segmentation/BoundaryRelationsMeanCurvature.h"

namespace v4r {

/************************************************************************************
 * Constructor/Destructor
 */

BoundaryRelationsMeanCurvature::BoundaryRelationsMeanCurvature() : BoundaryRelationsBase() {}

BoundaryRelationsMeanCurvature::~BoundaryRelationsMeanCurvature() {}

v4r::meanVal BoundaryRelationsMeanCurvature::compute() {
  //@ep: TODO check reconditions
  if (!have_cloud) {
    throw std::invalid_argument("[BoundaryRelationsMeanCurvature::compute] no input cloud set.");
  }

  if (!have_normals) {
    throw std::invalid_argument("[BoundaryRelationsMeanCurvature::compute] no normals set.");
  }

  if (!have_boundary) {
    throw std::invalid_argument("[BoundaryRelationsMeanCurvature::compute] no input border set.");
  }

  v4r::meanVal meanCurvature;

  if (boundary.size() <= 0) {
    LOG(WARNING) << "Boundary size is 0. This means that constants "
                    "are different everywhere!";
    meanCurvature.mean = 0;
    meanCurvature.stddev = 0;
    return meanCurvature;
  }

  int boundaryLength = boundary.size();

  // calculate mean depth
  double totalCurvature = 0.;
  double totalCurvatureStdDev = 0.;
  std::vector<double> valuesCurvature;
  valuesCurvature.reserve(boundaryLength);
  for (unsigned int i = 0; i < boundary.size(); i++) {
    pcl::PointXYZRGB p1 = cloud->points.at(boundary.at(i).idx1);
    pcl::PointXYZRGB p2 = cloud->points.at(boundary.at(i).idx2);

    if (checkNaN(p1) || checkNaN(p2)) {
      boundaryLength--;
      continue;
    }

    cv::Vec3f p0n;
    p0n[0] = normals->points.at(boundary.at(i).idx1).normal_x;
    p0n[1] = normals->points.at(boundary.at(i).idx1).normal_y;
    p0n[2] = normals->points.at(boundary.at(i).idx1).normal_z;

    cv::Vec3f p1n;
    p1n[0] = normals->points.at(boundary.at(i).idx2).normal_x;
    p1n[1] = normals->points.at(boundary.at(i).idx2).normal_y;
    p1n[2] = normals->points.at(boundary.at(i).idx2).normal_z;

    //@ep: BUG this section is wrong, see above
    cv::Vec3f pp;
    if (boundary.at(i).direction == 0) {
      pp[0] = -1.0;
      pp[1] = 0.0;
      pp[2] = 0.0;
    } else if (boundary.at(i).direction == 1) {
      pp[0] = -1.0;
      pp[1] = -1.0;
      pp[2] = 0.0;
    } else if (boundary.at(i).direction == 2) {
      pp[0] = 0.0;
      pp[1] = -1.0;
      pp[2] = 0.0;
    } else if (boundary.at(i).direction == 3) {
      pp[0] = 1.0;
      pp[1] = -1.0;
      pp[2] = 0.0;
    }
    cv::Vec3f pp_dir = cv::normalize(pp);
    //@ep: end of BUG

    double a_p0_pp = acos(p0n.ddot(pp_dir));
    pp_dir = -pp_dir;  // invert direction between points
    double a_p1_pp = acos(p1n.ddot(pp_dir));

    //@ep: BUG the next line is wrong, see above
    double curvature = a_p0_pp + a_p1_pp - M_PI;

    valuesCurvature.push_back(curvature);
    totalCurvature += curvature;
  }

  // normalize curvature sum and calculate curvature variance
  //@ep: this should be separate function in the utils
  if (boundaryLength > 0) {
    totalCurvature /= boundaryLength;
    for (unsigned i = 0; i < valuesCurvature.size(); i++) {
      //@ep: BUG why is it standard deviation???
      totalCurvatureStdDev += fabs(valuesCurvature.at(i) - totalCurvature);
    }
  } else {
    LOG(WARNING) << "Number of valid points is zero: totalCurvature: " << totalCurvature;
    totalCurvature = 0.;
    totalCurvatureStdDev = 0.;
  }

  meanCurvature.mean = totalCurvature;
  meanCurvature.stddev = totalCurvatureStdDev;

  return meanCurvature;
}

}  // namespace v4r
