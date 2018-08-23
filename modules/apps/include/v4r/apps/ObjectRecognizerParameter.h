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
 * @file ObjectRecognizerParameter.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include <v4r/config.h>
#include <boost/program_options.hpp>

#if HAVE_V4R_RENDERING
#include <v4r/apps/ViewRenderer.h>
#endif

#include <v4r/apps/CloudSegmenter.h>
#include <v4r/apps/visualization.h>
#include <v4r/common/intrinsics.h>
#include <v4r/common/normals.h>
#include <v4r/core/macros.h>

#if HAVE_V4R_FEATURES_AKAZE_LOCAL_ESTIMATOR
#include <v4r/features/FeatureDetector_KD_AKAZE.h>
#endif

#include <v4r/features/FeatureDetector_KD_BRISK.h>
#include <v4r/features/FeatureDetector_KD_ORB.h>

#if HAVE_V4R_FEATURES_SIFT_LOCAL_ESTIMATOR
#include <v4r/features/FeatureDetector_KD_CVSIFT.h>
#endif

#include <v4r/features/FeatureDetector_KD_FAST_IMGD.h>
#include <v4r/features/FeatureDetector_K_HARRIS.h>
#include <v4r/features/FeatureDetector_K_MSER.h>
#include <v4r/features/global_concatenated.h>
#include <v4r/features/rops_local_estimator.h>
#include <v4r/features/shot_local_estimator.h>

#if HAVE_V4R_FEATURES_OPENCV_XFEATURES2D
#include <v4r/features/FeatureDetector_D_FREAK.h>
#include <v4r/features/FeatureDetector_KD_SURF.h>
#endif

#include <v4r/keypoints/types.h>
#include <v4r/ml/types.h>
#include <v4r/recognition/global_recognizer.h>
#include <v4r/recognition/hypotheses_verification_param.h>
#include <v4r/recognition/local_recognition_pipeline.h>
#include <v4r/recognition/multiview_recognizer.h>
#include <v4r/segmentation/types.h>

namespace po = boost::program_options;

namespace v4r {

namespace apps {

struct V4R_EXPORTS FeatureDetectorParameter {
#if HAVE_V4R_FEATURES_AKAZE_LOCAL_ESTIMATOR
  FeatureDetector_KD_AKAZE::Parameter akaze_;
#endif
#if HAVE_V4R_FEATURES_OPENCV_XFEATURES2D
  FeatureDetector_KD_SURF::Parameter surf_;
  FeatureDetector_D_FREAK::Parameter freak_;
#endif
  FeatureDetector_KD_BRISK::Parameter brisk_;
  FeatureDetector_K_MSER::Parameter mser_;
  FeatureDetector_KD_ORB::Parameter orb_;

  void init(boost::program_options::options_description &desc);
};

struct V4R_EXPORTS ObjectRecognizerParameter {
  Intrinsics cam_;
  LocalRecognitionPipelineParameter local_rec_pipeline_;
  GraphGeometricConsistencyGroupingParameter gc_;
  GlobalRecognizerParameter global_rec_;
  PCLVisualizationParams::Ptr visualization_;
  MultiviewRecognizerParameter multiview_;

#ifdef HAVE_V4R_RENDERING
  v4r::apps::ViewRendererParameter rendering_;
#endif

  FeatureDetectorParameter feat_params_;
  LocalRecognizerParameter shot_pipeline_;
  LocalRecognizerParameter local_2d_pipeline_;
  apps::CloudSegmenterParameter plane_filter_;
  HV_Parameter hv_;

  bool use_graph_based_gc_grouping_ = true;  ///< if true, uses graph-based geometric consistency grouping. Otherwise
                                             ///< normal PCL implemented based geometric consistency grouping

  // pipeline setup
  bool do_local_2d_ = true;  ///< enable local 2D pipeline
  bool do_shot_ = false;     ///< enable SHOT feature pipeline
  v4r::FeatureDetector::Type local_2D_feat_est_ =
      v4r::FeatureDetector::Type::KD_CVSIFT;  ///< feature descriptor for 2D local pipeline
  v4r::FeatureDetector::Type local_2D_feat_detector_ =
      v4r::FeatureDetector::Type::KD_CVSIFT;  ///< keypoint detector for 2D local pipeline

  SegmentationType segmentation_method_ = SegmentationType::ORGANIZED_CONNECTED_COMPONENTS;
  std::vector<int> global_feature_types_ =
      {};  // { FeatureType::ESF | FeatureType::SIMPLE_SHAPE | FeatureType::GLOBAL_COLOR,
  // FeatureType::ALEXNET }
  // /< Concatenate all feature descriptors which corresponding feature type bit
  /// id (v4r/features/types.h) is set in this variable. Each (outer) element
  /// will be a separate global recognition pipeline.
  std::vector<ClassifierType> classification_methods_ = {ClassifierType::SVM};
  KeypointType shot_keypoint_extractor_method_ =
      KeypointType::HARRIS3D;  ///< Keypoint extraction method used for SHOT features
  NormalEstimatorType normal_computation_method_ =
      NormalEstimatorType::PCL_INTEGRAL_NORMAL;                 ///< normal computation method
  std::vector<float> keypoint_support_radii_ = {0.04f, 0.08f};  ///< support radi used for describing SHOT features

  // filter parameter
  double chop_z_ = 3.f;  ///< Cut-off distance in meter
  bool remove_planes_ =
      true;  ///< if enabled, removes the dominant plane in the input cloud (given thera are at least N
  /// inliers)
  float plane_inlier_threshold_ = 0.02f;  ///< maximum inlier threshold in meter for plane inliers
  size_t min_plane_inliers_ = 20000;      ///< required inliers for plane to be removed
  bool remove_non_upright_objects_ =
      false;  ///< removes all objects that are not upright (requires to extract support plane)

  // multi-view parameters
  bool use_multiview_ = false;  ///< if true, transfers verified hypotheses across views
  bool use_multiview_hv_ =
      true;  ///< if true, verifies hypotheses against the registered scene cloud from all input views
  bool use_multiview_with_kp_correspondence_transfer_ =
      false;  ///< if true, transfers keypoints instead of full hypotheses
  ///(see Faeulhammer et al, ICRA 2015)
  bool use_change_detection_ =
      true;  ///< if true, uses change detection to find dynamic elements within observation period
  ///(only for multi-view recognition)
  float tolerance_for_cloud_diff_ = 0.02f;  ///< tolerance in meter for change detection's cloud differencing
  size_t min_points_for_hyp_removal_ =
      50;  ///< how many removed points must overlap hypothesis to be also considered removed
  size_t multiview_max_views_ =
      3;  ///< maximum number of views used for multi-view recognition (if more views are available,
  /// information from oldest views will be ignored)

  size_t icp_iterations_ =
      0;  ///< ICP iterations. Only used if hypotheses are not verified. Otherwise ICP is done inside HV

  bool skip_verification_ = false;     ///< if true, will only generate hypotheses but not verify them
  bool visualize_hv_go_cues_ = false;  ///< If set, visualizes cues computed at the hypothesis verification stage such
                                       ///< as inlier, outlier points. Mainly used for debugging.
  bool visualize_hv_model_cues_ = false;  ///< If set, visualizes the model cues in the hypotheses verification stage.
  bool visualize_hv_pairwise_cues_ =
      false;                          ///< If set, visualizes the pairwise cues in the hypotheses verification stage.
  bool visualize_keypoints_ = false;  ///< If set, visualizes detected keypoints.
  bool visualize_global_results_ = false;  ///< If set, visualizes segments and results from global pipeline.
  ObjRecoVisLayoutStyle vis_layout_ =
      ObjRecoVisLayoutStyle::FULL;  ///< defines the layout of the visualization (FULL... full, INTERMEDIATE... only
                                    ///< show input and verified, SIMPLE... show input, processed, and verified)

  ObjectRecognizerParameter() {
    visualization_.reset(new PCLVisualizationParams);
  }

  void validate();

  /**
   * @brief init parameters
   * @param command_line_arguments (according to Boost program options library)
   * @param section_name section name of program options
   */
  void init(boost::program_options::options_description &desc, const std::string &section_name = "or_multipipeline");
};
}  // namespace apps
}  // namespace v4r
