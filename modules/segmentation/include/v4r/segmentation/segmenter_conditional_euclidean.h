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
 * @file   segmenter_conditional_euclidean.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date   May, 2018
 * @brief
 *
 */

#pragma once

#include <v4r/common/pcl_utils.h>
#include <v4r/core/macros.h>
#include <v4r/segmentation/segmenter.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace v4r {

struct V4R_EXPORTS ConditionalEuclideanSegmenterParameter : public SegmenterParameter {
  using SegmenterParameter::cluster_tolerance_;
  using SegmenterParameter::max_cluster_size_;
  using SegmenterParameter::min_cluster_size_;
  bool z_adaptive_ = true;
};

/**
 * @brief Segmentation class that clusters points Euclidean based with a custom defined function
 * @tparam PointT
 */
template <typename PointT>
class V4R_EXPORTS ConditionalEuclideanSegmenter : public Segmenter<PointT> {
  using Segmenter<PointT>::clusters_;
  using Segmenter<PointT>::scene_;

  ConditionalEuclideanSegmenterParameter param_;

  /** \brief A pointer to the spatial search object */
  typename pcl::search::Search<PointT>::Ptr searcher_;

  /** \brief The condition function that needs to hold for clustering */
  boost::function<bool(const PointT &, const PointT &, float)> condition_function_;

 public:
  ConditionalEuclideanSegmenter(
      const ConditionalEuclideanSegmenterParameter &p = ConditionalEuclideanSegmenterParameter())
  : param_(p) {}

  bool getRequiresNormals() const {
    return true;  /// TODO: This depends on condition function
  }

  /** \brief Set the condition that needs to hold for neighboring points to be considered part of the same cluster.
   * This is an overloaded function provided for convenience. See the documentation for setConditionFunction(). */
  void setConditionFunction(boost::function<bool(const PointT &, const PointT &, float)> condition_function) {
    condition_function_ = condition_function;
  }

  void segment();

  typedef std::shared_ptr<ConditionalEuclideanSegmenter<PointT>> Ptr;
  typedef std::shared_ptr<ConditionalEuclideanSegmenter<PointT> const> ConstPtr;
};
}  // namespace v4r
