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
 * @file local_recognition_pipeline.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <v4r/common/graph_geometric_consistency.h>
#include <v4r/io/filesystem.h>
#include <v4r/recognition/local_feature_matching.h>
#include <v4r/recognition/recognition_pipeline.h>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <omp.h>
#include <pcl/recognition/cg/correspondence_grouping.h>

namespace po = boost::program_options;

namespace v4r {

struct V4R_EXPORTS LocalRecognitionPipelineParameter {
  bool merge_close_hypotheses_ =
      true;  ///< if true, close correspondence clusters (object hypotheses) of the same object
             /// model are merged together and this big cluster is refined
  float merge_close_hypotheses_dist_ =
      0.02f;  ///< defines the maximum distance of the centroids in meter for clusters to be
              /// merged together
  float merge_close_hypotheses_angle_ =
      10.f;                  ///< defines the maximum angle in degrees for clusters to be merged together
  float min_dist_ = 0.005f;  ///< minimum distance two points need to be apart to be counted as redundant
  float max_dotp_ =
      0.95f;  ///< maximum dot-product between the surface normals of two oriented points to be counted redundant
  size_t max_num_hypotheses_per_object_ =
      0;  ///< maximum number of hypotheses that are generated for a certain object. If
  /// more hypotheses are found, the ones with the fewest number of keypoint correspondences will be discarded.
  /// No limit for 0

  void init(boost::program_options::options_description &desc, const std::string &section_name = "local_rec") {
    desc.add_options()(
        (section_name + ".merge_close_hypotheses").c_str(),
        po::value<bool>(&merge_close_hypotheses_)->default_value(merge_close_hypotheses_),
        "if true, close correspondence clusters "
        "(object hypotheses) of the same object model are merged together and this big cluster is refined");
    desc.add_options()((section_name + ".merge_close_hypotheses_dist").c_str(),
                       po::value<float>(&merge_close_hypotheses_dist_)->default_value(merge_close_hypotheses_dist_),
                       "defines the maximum distance of the centroids "
                       "in meter for clusters to be merged together (if enabled)");
    desc.add_options()((section_name + ".merge_close_hypotheses_angle").c_str(),
                       po::value<float>(&merge_close_hypotheses_angle_)->default_value(merge_close_hypotheses_angle_),
                       "defines the maximum angle in degrees for "
                       "clusters to be merged together (if enabled)");
    desc.add_options()((section_name + ".min_dist").c_str(), po::value<float>(&min_dist_)->default_value(min_dist_),
                       "minimum distance two points need to be apart to be counted as redundant");
    desc.add_options()(
        (section_name + ".max_dotp").c_str(), po::value<float>(&max_dotp_)->default_value(max_dotp_),
        "maximum dot-product between the surface normals of two oriented points to be counted redundant");
    desc.add_options()(
        (section_name + ".max_num_hypotheses_per_object").c_str(),
        po::value<size_t>(&max_num_hypotheses_per_object_)->default_value(max_num_hypotheses_per_object_),
        "maximum number of hypotheses that are generated for a certain "
        "object. If more hypotheses are found, the ones with the fewest number of "
        "keypoint correspondences will be discarded. No limit for 0");
  }
};

/**
 * @brief This class merges keypoint correspondences from several local recognizers and generate object hypotheses.
 * @author Thomas Faeulhammer
 * @date Jan 2017
 */
template <typename PointT>
class V4R_EXPORTS LocalRecognitionPipeline : public RecognitionPipeline<PointT> {
 private:
  using RecognitionPipeline<PointT>::elapsed_time_;
  using RecognitionPipeline<PointT>::m_db_;
  using RecognitionPipeline<PointT>::normal_estimator_;
  using RecognitionPipeline<PointT>::obj_hypotheses_;
  using RecognitionPipeline<PointT>::scene_;
  using RecognitionPipeline<PointT>::scene_normals_;
  using RecognitionPipeline<PointT>::vis_param_;

  std::vector<typename LocalFeatureMatcher<PointT>::Ptr>
      local_feature_matchers_;  ///< set of local recognizer generating keypoint correspondences

  typename std::shared_ptr<pcl::CorrespondenceGrouping<pcl::PointXYZ, pcl::PointXYZ>>
      cg_algorithm_;  ///< algorithm for correspondence grouping
  std::map<std::string, LocalObjectHypothesis<PointT>> local_obj_hypotheses_;  ///< stores feature correspondences
  std::map<std::string, typename LocalObjectModel::ConstPtr>
      model_keypoints_;  ///< object model database used for local recognition
  std::vector<std::map<std::string, size_t>> model_kp_idx_range_start_;  ///< since keypoints are coming from multiple
                                                                         /// local recognizer, we need to store which
  /// range belongs to which recognizer. This
  /// variable is the starting parting t

  LocalRecognitionPipelineParameter param_;

  /**
   * @brief correspondenceGrouping
   */
  void correspondenceGrouping(const std::vector<std::string> &model_ids_to_search);

  /**
   * @brief recognize
   * @param model_ids_to_search object model ids to search for
   */
  void do_recognize(const std::vector<std::string> &model_ids_to_search) override;

  bool generate_hypotheses_;  ///< if true, cluster correspondences with respect to geometeric consistency and estimates
                              /// pose by SVD

  void doInit(const bf::path &trained_dir, bool force_retrain,
              const std::vector<std::string> &object_instances_to_load) override;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LocalRecognitionPipeline(const LocalRecognitionPipelineParameter &p = LocalRecognitionPipelineParameter())
  : param_(p), generate_hypotheses_(true) {}

  /**
   * @brief setCGAlgorithm
   * @param alg
   */
  void setCGAlgorithm(const std::shared_ptr<pcl::CorrespondenceGrouping<pcl::PointXYZ, pcl::PointXYZ>> &alg) {
    cg_algorithm_ = alg;
  }

  /**
   * @brief addRecognizer
   * @param l_feature_matcher local feature matcher
   */
  void addLocalFeatureMatcher(const typename LocalFeatureMatcher<PointT>::Ptr &l_feature_matcher) {
    local_feature_matchers_.push_back(l_feature_matcher);
  }

  /**
   * @brief needNormals
   * @return
   */
  bool needNormals() const override {
    for (size_t r_id = 0; r_id < local_feature_matchers_.size(); r_id++) {
      if (local_feature_matchers_[r_id]->needNormals())
        return true;
    }

    // Graph-based correspondence grouping requires normals but interface does not exist in base class - so need to try
    // pointer casting
    typename GraphGeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>::Ptr gcg_algorithm =
        std::dynamic_pointer_cast<GraphGeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>>(cg_algorithm_);
    if (gcg_algorithm)
      return true;

    return false;
  }

  /**
   * @brief getFeatureType
   * @return
   */
  size_t getFeatureType() const override {
    size_t feat_type = 0;
    for (size_t r_id = 0; r_id < local_feature_matchers_.size(); r_id++)
      feat_type += local_feature_matchers_[r_id]->getFeatureType();

    return feat_type;
  }

  bool requiresSegmentation() const override {
    return false;
  }

  /**
   * @brief getLocalObjectModelDatabase
   * @return local object model database
   */
  std::map<std::string, typename LocalObjectModel::ConstPtr> getLocalObjectModelDatabase() const {
    return model_keypoints_;
  }

  /**
   * @brief disableHypothesesGeneration does not run geometric consistency grouping and only finds keypoint
   * correspondences (e.g. when correspondence grouping is done outsided)
   */
  void disableHypothesesGeneration() {
    generate_hypotheses_ = false;
  }

  /**
   * @brief getKeypointCorrespondences get feature matches for all objects in the model database
   * @return
   */
  std::map<std::string, LocalObjectHypothesis<PointT>> getKeypointCorrespondences() const {
    return local_obj_hypotheses_;
  }

  typedef std::shared_ptr<LocalRecognitionPipeline<PointT>> Ptr;
  typedef std::shared_ptr<LocalRecognitionPipeline<PointT> const> ConstPtr;
};
}  // namespace v4r
