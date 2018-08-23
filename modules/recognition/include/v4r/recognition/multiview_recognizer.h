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
 * @file multiview_recognizer.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date January, 2017
 * @brief multiview object instance recognizer
 *
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/recognition/cg/correspondence_grouping.h>
#include <v4r/config.h>
#include <v4r/recognition/local_feature_matching.h>
#include <v4r/recognition/recognition_pipeline.h>
#include <boost/program_options.hpp>
#include <vector>

namespace v4r {

struct V4R_EXPORTS MultiviewRecognizerParameter {
  bool transfer_only_verified_hypotheses_ = true;
  size_t max_views_ = 3;  ///< maximum views to take into account
  bool transfer_keypoint_correspondences_ =
      false;  ///< if true, transfers keypoint correspondences instead of full hypotheses
              ///(requires correspondence grouping)
  bool merge_close_hypotheses_ =
      true;  ///< if true, close correspondence clusters (object hypotheses) of the same object
             /// model are merged together and this big cluster is refined
  float merge_close_hypotheses_dist_ =
      0.02f;  ///< defines the maximum distance of the centroids in meter for clusters to be
              /// merged together
  float merge_close_hypotheses_angle_ =
      10.f;                 ///< defines the maximum angle in degrees for clusters to be merged together
  float min_dist_ = 0.01f;  ///< minimum distance two points need to be apart to be counted as redundant
  float max_dotp_ =
      0.95f;  ///< maximum dot-product between the surface normals of two oriented points to be counted redundant
  bool visualize_ = true;

  /**
   * @brief init parameters
   * @param command_line_arguments (according to Boost program options library)
   * @param section_name section name of program options
   */
  void init(boost::program_options::options_description &desc, const std::string &section_name = "mv_rec");
};

template <typename PointT>
class V4R_EXPORTS MultiviewRecognizer : public RecognitionPipeline<PointT> {
 private:
  using RecognitionPipeline<PointT>::scene_;
  using RecognitionPipeline<PointT>::scene_normals_;
  using RecognitionPipeline<PointT>::m_db_;
  using RecognitionPipeline<PointT>::obj_hypotheses_;
  using RecognitionPipeline<PointT>::table_plane_;
  using RecognitionPipeline<PointT>::table_plane_set_;

  typename RecognitionPipeline<PointT>::Ptr recognition_pipeline_;

  MultiviewRecognizerParameter param_;

  struct View {
    Eigen::Matrix4f camera_pose_;  ///< camera pose of the view which aligns cloud in registered cloud when multiplied
    std::vector<ObjectHypothesesGroup> obj_hypotheses_;  ///< generated object hypotheses

    std::map<std::string, LocalObjectHypothesis<PointT>> local_obj_hypotheses_;  ///< stores feature correspondences
    std::map<std::string, typename LocalObjectModel::ConstPtr>
        model_keypoints_;  ///< object model database used for local recognition
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene_cloud_xyz_;
    pcl::PointCloud<pcl::Normal>::ConstPtr scene_cloud_normals_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    View() : camera_pose_(Eigen::Matrix4f::Identity()) {}
  };

  std::vector<View> views_;

  // for keypoint correspondence transfer
  typename std::shared_ptr<pcl::CorrespondenceGrouping<pcl::PointXYZ, pcl::PointXYZ>>
      cg_algorithm_;  ///< algorithm for correspondence grouping
  std::map<std::string, LocalObjectHypothesis<PointT>> local_obj_hypotheses_;  ///< stores feature correspondences
  std::map<std::string, typename LocalObjectModel::ConstPtr>
      model_keypoints_;  ///< object model database used for local recognition

  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_xyz_merged_;
  pcl::PointCloud<pcl::Normal>::Ptr scene_cloud_normals_merged_;

  void visualize();

  void doInit(const bf::path &trained_dir, bool retrain, const std::vector<std::string> &object_instances_to_load);

  /**
   * @brief recognize
   */
  void do_recognize(const std::vector<std::string> &model_ids_to_search);

  void correspondenceGrouping(const std::vector<std::string> &model_ids_to_search);

 public:
  MultiviewRecognizer(const MultiviewRecognizerParameter &p = MultiviewRecognizerParameter()) : param_(p) {}

  /**
   * @brief oh_tmp
   * @param rec recognition pipeline (local or global)
   */
  void setSingleViewRecognitionPipeline(typename RecognitionPipeline<PointT>::Ptr &rec) {
    recognition_pipeline_ = rec;
  }

  /**
   * @brief needNormals
   * @return true if normals are needed, false otherwise
   */
  bool needNormals() const {
    return recognition_pipeline_->needNormals();
  }

  /**
   * @brief getFeatureType
   * @return
   */
  size_t getFeatureType() const {
    return recognition_pipeline_->getFeatureType();
  }

  /**
   * @brief requiresSegmentation
   * @return
   */
  bool requiresSegmentation() const {
    return recognition_pipeline_->requiresSegmentation();
  }

  void clear() {
    views_.clear();
  }

  /**
   * @brief setCGAlgorithm set the correspondence grouping algorithm
   * @param alg shared ptr to correspondence grouping algorithm
   */
  void setCGAlgorithm(const std::shared_ptr<pcl::CorrespondenceGrouping<pcl::PointXYZ, pcl::PointXYZ>> &alg) {
    cg_algorithm_ = alg;
  }

  typedef std::shared_ptr<MultiviewRecognizer<PointT>> Ptr;
  typedef std::shared_ptr<MultiviewRecognizer<PointT> const> ConstPtr;
};
}  // namespace v4r
