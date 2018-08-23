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
 * @file ObjectRecognizer.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <v4r/apps/CloudSegmenter.h>
#include <v4r/apps/ObjectRecognizerParameter.h>
#include <v4r/apps/visualization.h>
#include <v4r/common/normals.h>
#include <v4r/config.h>
#include <v4r/core/macros.h>
#include <v4r/io/filesystem.h>
#include <v4r/recognition/hypotheses_verification.h>
#include <v4r/recognition/local_recognition_pipeline.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>
#include <boost/serialization/vector.hpp>

namespace bf = boost::filesystem;

namespace v4r {

namespace apps {

/**
 * @brief Class that sets up multi-pipeline object recognizer
 * @author Thomas Faeulhammer
 * @tparam PointT
 */
template <typename PointT>
class V4R_EXPORTS ObjectRecognizer {
 private:
  typename v4r::RecognitionPipeline<PointT>::Ptr mrec_;                             ///< multi-pipeline recognizer
  typename v4r::LocalRecognitionPipeline<PointT>::Ptr local_recognition_pipeline_;  ///< local recognition pipeline
                                                                                    ///(member variable just because of
                                                                                    /// visualization of keypoints)
  typename v4r::HypothesisVerification<PointT>::Ptr hv_;                            ///< hypothesis verification object
  typename v4r::NormalEstimator<PointT>::Ptr
      normal_estimator_;  ///< normal estimator used for computing surface normals (currently only used at training)

  typename v4r::ObjectRecognitionVisualizer<PointT>::Ptr rec_vis_;  ///< visualization object

  typename v4r::apps::CloudSegmenter<PointT>::Ptr plane_extractor_;  ///< cloud segmenter for plane removal (if enabled)

  bool visualize_;  ///< if true, visualizes objects
  bf::path models_dir_;

  ObjectRecognizerParameter param_;  ///< parameters for object recognition

  typename Source<PointT>::Ptr model_database_;  ///< object model database

  void setupLocal2DPipeLine();
  FeatureDetector::Ptr getFeatureDetector(const v4r::FeatureDetector::Type &type) const;

  /**
   * @brief helper class for multi-view recognition system
   */
  class View {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typename pcl::PointCloud<PointT>::ConstPtr cloud_;
    typename pcl::PointCloud<PointT>::Ptr processed_cloud_;
    typename pcl::PointCloud<PointT>::Ptr removed_points_;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
    std::vector<std::vector<float>> pt_properties_;
    Eigen::Matrix4f camera_pose_;
  };
  std::vector<View> views_;  ///< all views in sequence

#if HAVE_V4R_CHANGE_DETECTION
  /**
   * @brief detectChanges detect changes in multi-view sequence (e.g. objects removed or added to the scene within
   * observation period)
   * @param v current view
   */
  void detectChanges(View &v);
#endif

  typename pcl::PointCloud<PointT>::Ptr registered_scene_cloud_;  ///< registered point cloud of all processed input
                                                                  /// clouds in common camera reference frame

  std::vector<std::pair<std::string, float>>
      elapsed_time_;  ///< measurements of computation times for various components

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ObjectRecognizer() : visualize_(false) {}

  /**
   * @brief initialize initialize Object recognizer (sets up model database, recognition pipeline and hypotheses
   * verification)
   * @param argc
   * @param argv
   */
  void initialize(int argc, char **argv) {
    std::vector<std::string> arguments(argv + 1, argv + argc);
    initialize(arguments);
  }

  /**
   * @brief initialize initialize Object recognizer (sets up model database, recognition pipeline and hypotheses
   * verification)
   * @param arguments
   */
  void initialize(std::vector<std::string> &command_line_arguments,
                  const boost::filesystem::path &config_folder = bf::path("cfg"));

  /**
   * @brief recognize recognize objects in point cloud together with their 6DoF pose
   * @param cloud (organized) point cloud
   * @param obj_models_to_search object model identities to detect. If empty, all object models in database will be
   * tried to detect
   * @return
   */
  std::vector<ObjectHypothesesGroup> recognize(
      const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
      const std::vector<std::string> &obj_models_to_search = std::vector<std::string>());

  /**
   * @brief get point cloud of object model
   * @param model_name identity of object model to return
   * @param resolution_mm  resolution of point cloud model in milli meter
   * @return point cloud of object model or nullptr if it does not exist
   */
  typename pcl::PointCloud<PointT>::ConstPtr getModel(const std::string &model_name, int resolution_mm) const;

  /**
   * @brief get path to models directory
   * @return
   */
  bf::path getModelsDir() const {
    return models_dir_;
  }

  /**
   * @brief set path to models directory
   * @param dir
   */
  void setModelsDir(const bf::path &dir) {
    models_dir_ = dir;
  }

  /**
   * @brief getElapsedTimes
   * @return compuation time measurements for various components
   */
  std::vector<std::pair<std::string, float>> getElapsedTimes() const {
    return elapsed_time_;
  }

  /**
   * @brief getParam get recognition parameter
   * @return parameter
   */
  ObjectRecognizerParameter getParam() const {
    return param_;
  }

  /**
   * @brief getCamera get camera intrinsic parameters
   * @return camera intrinsic parameter
   */
  Intrinsics getCameraIntrinsics() const {
    return param_.cam_;
  }

  /**
   * @brief setCamera set the camera intrinsics used for z-buffering
   * @param cam camera intrinsic parameters
   */
  void setCameraIntrinsics(const Intrinsics &cam) {
    param_.cam_ = cam;

    if (hv_) {
      hv_->setCameraIntrinsics(param_.cam_);
    }
  }

  /**
   * @brief resetMultiView resets all state variables of the multi-view and initializes a new multi-view sequence
   */
  void resetMultiView();
};
}  // namespace apps
}  // namespace v4r
