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
 * @file hypotheses_verifcation_param.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief Parameters for object hypotheses verification
 *
 */

#pragma once

#include <v4r/common/color_comparison.h>
#include <v4r/common/normal_estimator.h>
#include <v4r/core/macros.h>
#include <v4r/io/filesystem.h>
#include <boost/program_options.hpp>
#include <fstream>
namespace po = boost::program_options;

namespace v4r {

struct V4R_EXPORTS HV_Parameter {
  int resolution_mm_ = 5;  ///< The resolution of models and scene used to verify hypotheses (in milli meters)
  float occlusion_thres_ =
      0.01f;  ///< Threshold for a point to be considered occluded when model points are back-projected to
              /// the scene ( depends e.g. on sensor noise)
  int smoothing_radius_ =
      2;  ///< radius in pixel used for smoothing the visible image mask of an object hypotheses (used
          /// for computing pairwise intersection)
  bool do_smoothing_ =
      true;  ///< if true, smoothes the silhouette of the reproject object hypotheses (used for computing
             /// pairwise intersection)
  bool do_erosion_ =
      true;  ///< if true, performs erosion on the silhouette of the reproject object hypotheses. This should
             /// avoid a pairwise cost for touching objects (used for computing pairwise intersection)
  int erosion_radius_ = 4;  ///< erosion radius in px (used for computing pairwise intersection)

  // ICP stuff
  int icp_iterations_ = 10;               ///< number of icp iterations for pose refinement
  float icp_max_correspondence_ = 0.02f;  ///< the maximum distance threshold between a point and its nearest neighbor
                                          /// correspondent in order to be considered in the ICP alignment process

  float min_Euclidean_dist_between_centroids_ =
      0.01f;  ///< minimum Euclidean distances in meter between the centroids of two
              ///< hypotheses of the same object model to be treated
              ///< separately
  float min_angular_degree_dist_between_hypotheses_ = 5.f;  ///< minimum angular distance in degree between two
                                                            ///< hypotheses of the same object model to be treated
                                                            ///< separately

  float inlier_threshold_xyz_ = 0.01f;  ///< inlier distance in meters between model and scene point
  float inlier_threshold_normals_dotp_ =
      0.99f;  ///< inner product for which the fit of model and scene surface normals is
              /// exactly half
  float inlier_threshold_color_ =
      30.f;  ///< allowed chrominance (AB channel of LAB color space) variance for a point of an
             /// object hypotheses to be considered explained by a corresponding scene point
  ///(between 0 and 1, the higher the fewer objects get rejected)

  float sigma_xyz_ = 0.01f;      ///< variance for normals between model and scene
  float sigma_normals_ = 0.05f;  ///< variance for normals between model and scene
  float sigma_color_ =
      20.f;  ///< allowed illumination (L channel of LAB color space) variance for a point of an object
             /// hypotheses to be considered explained by a corresponding scene point (between 0 and 1, the
  /// higher the fewer objects get rejected)

  float w_xyz_ = 1.f / 3.f;      ///< weighting factor for 3D fitness
  float w_normals_ = 1.f / 3.f;  ///< weighting factor for normal fitness
  float w_color_ = 1.f / 3.f;    ///< weighting factor for color fitness

  NormalEstimatorType normal_method_ = NormalEstimatorType::PCL_DEFAULT;  ///< method used for computing the normals of
                                                                          ///< the downsampled scene point cloud
                                                                          ///< (defined by the
                                                                          /// V4R Library)
  bool ignore_color_even_if_exists_ = false;  ///< if true, only checks 3D Eucliden distance of neighboring points
  int max_iterations_ =
      5000;  ///< max iterations the optimization strategy explores local neighborhoods before stopping
             /// because the cost does not decrease.
  float clutter_regularizer_ =
      0.1f;  ///< The penalty multiplier used to penalize unexplained scene points within the clutter
             /// influence radius <i>radius_neighborhood_clutter_</i> of an explained scene point when
  /// they belong to the same smooth segment.
  bool use_histogram_specification_ =
      false;  ///< if true, tries to globally match brightness (L channel of LAB color space) of
              /// visible hypothesis cloud to brightness of nearby scene points. It does so by
  /// computing the L channel histograms for both clouds and shifting it to maximize
  /// histogram intersection.
  ColorComparisonMethod color_comparison_method_ =
      ColorComparisonMethod::CIEDE2000;  ///< method used for color comparison (CIE76, CIE94, CIEDE2000)
  float min_visible_ratio_ =
      0.15f;  ///< defines how much of the object has to be visible in order to be included in the
              /// verification stage

  // Euclidean smooth segmenation
  bool check_smooth_clusters_ =
      true;  ///< if true, checks if hypotheses explain whole smooth regions of input cloud (if they
             /// only partially explain one smooth region, the solution is rejected)
  float eps_angle_threshold_deg_ = 5.f;  ///< angle threshold in degree to cluster two neighboring points together
  float curvature_threshold_ = 0.04f;    ///< curvature threshold to allow clustering of two points (points with surface
                                         /// curvatures higher than this threshold are skipped)
  float cluster_tolerance_ =
      0.01f;  ///< cluster tolerance in meters for points to be clustered together when checking smooth clusters
  size_t min_points_ = 100;                   ///< minimum number of points for a smooth region to be extracted
  float min_ratio_cluster_explained_ = 0.5f;  ///< defines the minimum ratio a smooth cluster has to be explained by the
                                              /// visible points (given there are at least 100 points)
  bool z_adaptive_ =
      true;  ///< if true, scales the smooth segmentation parameters linear with distance (constant till 1m at
             /// the given parameters)
  size_t min_pts_smooth_cluster_to_be_epxlained_ =
      50;  ///< minimum number of points a cluster need to be explained by model
  /// points to be considered for a check (avoids the fact that boundary
  /// points of a smooth region can be close to an object)

  float min_fitness_ = 0.2f;  ///< points which have a lower fitness score will be defined as \"outlier\" (this value
                              /// corresponds to models that are fully visible)
  float min_fitness_high_ =
      0.4f;  ///< points which have a lower fitness score will be defined as \"outlier\" (this value
             /// corresponds to models that just visible by the defined min_visible_threshold)
  float min_dotproduct_model_normal_to_viewray_ =
      0.2f;  ///< surfaces which point are oriented away from the viewray will be
  /// discarded if the absolute dotproduct between the surface normal and
  /// the viewray is smaller than this threshold. This should ignore
  /// points for further fitness check which are very sensitive to small
  /// rotation changes.
  float min_px_distance_to_image_boundary_ =
      3.f;  ///< minimum distance in pixel a re-projected point needs to have to the
            /// image boundary

  /**
   * @brief init parameters
   * @param command_line_arguments (according to Boost program options library)
   * @param section_name section name of program options
   * @return unused parameters (given parameters that were not used in this initialization call)
   */
  void init(boost::program_options::options_description &desc, const std::string &section_name = "hv");
};
}  // namespace v4r
