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
 * @file AddGroundTruth.h
 * @author Richtsfeld, Potapova
 * @date Dezember 2012
 * @version 0.1
 * @brief Add ground truth to relations.
 */

#ifndef ADD_GROUND_TRUTH_H
#define ADD_GROUND_TRUTH_H

#include <stdio.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "opencv2/opencv.hpp"

#include "v4r/attention_segmentation/SurfaceModel.h"

namespace v4r {

class AddGroundTruth {
 public:
 private:
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud;  ///< Input cloud
  bool have_cloud;
  int width, height;

  std::vector<SurfaceModel::Ptr> surfaces;
  bool have_surfaces;

  std::vector<Relation> relations;
  bool have_relations;

  bool computed;

 public:
  AddGroundTruth();
  ~AddGroundTruth();

  /** Set input point cloud **/
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud);

  /** Set relations **/
  void setSurfaces(std::vector<SurfaceModel::Ptr> _surfaces);

  /** Set relations **/
  void setRelations(std::vector<v4r::Relation> _relations);

  /** Return modified surfaces **/
  inline std::vector<SurfaceModel::Ptr> getSurfaces();

  /** Return modified surfaces **/
  inline std::vector<Relation> getRelations();

  /** Add ground truth of 'type' to the relations **/
  void compute(int type = 1);
};

inline std::vector<SurfaceModel::Ptr> AddGroundTruth::getSurfaces() {
  return surfaces;
}

inline std::vector<Relation> AddGroundTruth::getRelations() {
  return relations;
}

}  // namespace v4r

#endif
