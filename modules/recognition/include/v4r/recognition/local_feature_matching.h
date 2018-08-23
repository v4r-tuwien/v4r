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
 * @file local_feature_matching.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2012
 * @brief
 *
 */

#pragma once

#include <pcl/common/common.h>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/program_options.hpp>
#include <boost/serialization/serialization.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/flann/flann.hpp>

#include <v4r/common/metrics.h>
#include <v4r/common/normals.h>
#include <v4r/common/pcl_visualization_utils.h>
#include <v4r/features/local_estimator.h>
#include <v4r/features/types.h>
#include <v4r/io/filesystem.h>
#include <v4r/keypoints/keypoint_extractor.h>
#include <v4r/recognition/local_rec_object_hypotheses.h>
#include <v4r/recognition/source.h>

namespace v4r {

struct V4R_EXPORTS LocalRecognizerParameter {
  // parameters for feature matching
  int kdtree_splits_ = 512;   ///< kdtree splits
  int kdtree_num_trees_ = 4;  ///< number of trees for FLANN approximate nearest neighbor search
  int kdtree_search_checks_ =
      128;          ///< The number of times the tree(s) in the index should be recursively traversed. A
                    ///< higher value for this parameter would give better search precision, but also take
                    ///< more time.
  size_t knn_ = 1;  ///< nearest neighbors to search for when checking feature descriptions of the scene
  int lsh_index_table_number_ = 20;      ///< Number of hash tables to use, usually 10-30
  int lsh_index_key_index_ = 10;         ///< key bits, usually 10-20
  int lsh_index_multi_probe_level_ = 2;  ///< controls how neighboring buckets are searched. Recommended value is 2. If
                                         ///< set to 0, the algorithm will degenerate into non-multiprobe LSH
  float max_descriptor_distance_ = std::numeric_limits<float>::max();  ///< maximum distance of the descriptor in the
                                                                       ///< respective norm (L1 or L2) to create a
                                                                       /// correspondence
  float correspondence_distance_weight_ =
      1.f;  ///< weight factor for correspondences distances. This is done to favour
            /// correspondences from different pipelines that are more reliable than other
  ///(SIFT and SHOT corr. simultaneously fed into CG)

  bool use_brute_force_matching_ =
      false;  ///< if true, runs brute force feature matching. Otherwise, uses FLANN approximate nearest neighbor search
  bool cross_check_ = false;  ///< If it is false, this is will be default BFMatcher behaviour when it finds the k
                              ///< nearest neighbors for each query descriptor. If crossCheck==true, then the knnMatch()
                              ///< method with k=1 will only return pairs (i,j) such that for i-th query descriptor the
                              ///< j-th descriptor in the matcher's collection is the nearest and vice versa, i.e. the
                              ///< BFMatcher will only return consistent pairs. Such technique usually produces best
                              ///< results with minimal number of outliers when there are enough matches. This is
                              ///< alternative to the ratio test, used by D. Lowe in SIFT paper.

  DistanceMetric distance_metric_ =
      DistanceMetric::L1;  ///< defines the norm used for feature matching (1... L1 norm, 2... L2 norm, 3... ChiSquare,
                           /// 4... Hellinger)
  float max_keypoint_distance_z_ =
      std::numeric_limits<float>::max();  ///< maximum distance of an extracted keypoint to be accepted

  // parameters for plane filter
  bool filter_planar_ = false;  ///< Filter keypoints with a planar surface
  int min_plane_size_ =
      1000;  ///< Minimum number of points for a plane to be checked if filter only points above table plane
  int planar_computation_method_;  ///< defines the method used to check for planar points. 0... based on curvate value
                                   /// after normalestimationomp, 1... with eigenvalue check of scatter matrix
  float planar_support_radius_ = 0.04f;  ///< Radius used to check keypoints for planarity.
  float threshold_planar_ = 0.02f;  ///< threshold ratio used for deciding if patch is planar. Ratio defined as largest
                                    /// eigenvalue to all others.

  // parameters for depth-discontinuity filter
  int filter_border_pts_ = 7;  ///< Filter keypoints at the boundary (value according to the edge types defined in
                               /// pcl::OrganizedEdgeBase (EDGELABEL_OCCLUDING  | EDGELABEL_OCCLUDED |
  /// EDGELABEL_NAN_BOUNDARY))
  int boundary_width_ = 5;  ///< Width in pixel of the depth discontinuity

  float required_viewpoint_change_deg_ =
      10.f;  ///< required viewpoint change in degree for a new training view to be used for
             /// feature extraction. Training views will be sorted incrementally by their
  /// filename and if the camera pose of a training view is close to the camera
  /// pose of an already existing training view, it will be discarded for
  /// training.

  bool train_on_individual_views_ =
      true;  ///< if true, extracts features from each view of the object model. Otherwise will
             /// use the full 3d cloud

  /**
   * @brief init parameters
   * @param command_line_arguments (according to Boost program options library)
   * @param section_name section name of program options
   */
  void init(boost::program_options::options_description &desc, const std::string &section_name = "lcl_ftr_match");
};

/**
 * @brief The LocalObjectModel class stores information about the object model related to local feature extraction
 */
class V4R_EXPORTS LocalObjectModel {
 public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_;  ///< all extracted keypoints of the object model
  pcl::PointCloud<pcl::Normal>::Ptr
      kp_normals_;  ///< normals associated to each extracted keypoints of the object model

  LocalObjectModel() {
    keypoints_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    kp_normals_.reset(new pcl::PointCloud<pcl::Normal>);
  }

  typedef std::shared_ptr<LocalObjectModel> Ptr;
  typedef std::shared_ptr<LocalObjectModel const> ConstPtr;
};

class V4R_EXPORTS LocalObjectModelDatabase {
 public:
  std::map<std::string, typename LocalObjectModel::ConstPtr>
      l_obj_models_;  ///< information about object models for each model id

  typedef std::shared_ptr<LocalObjectModelDatabase> Ptr;
  typedef std::shared_ptr<LocalObjectModelDatabase const> ConstPtr;

  std::shared_ptr<cv::DescriptorMatcher> matcher_;

  /**
   * @brief The flann_model class stores for each signature to which model and which keypoint it belongs to
   */
  struct flann_model {
    std::string model_id_;
    size_t keypoint_id_;
  };

  std::vector<flann_model> flann_models_;
};

/**
 * \brief Object recognition + 6DOF pose based on local features, GC and HV
 * Contains keypoints/local features computation, matching using FLANN,
 * point-to-point correspondence grouping, pose refinement and hypotheses verification
 * Available features: SHOT, FPFH
 * See apps/3d_rec_framework/tools/apps for usage
 * \author Aitor Aldoma, Federico Tombari, Thomas Faeulhammer
 */
template <typename PointT>
class V4R_EXPORTS LocalFeatureMatcher {
 private:
  typedef std::vector<float> FeatureDescriptor;
  typedef int KeypointIndex;

  typename pcl::PointCloud<PointT>::ConstPtr scene_;  ///< Point cloud to be classified
  std::vector<int> indices_;  ///< segmented cloud to be recognized (if empty, all points will be processed)
  pcl::PointCloud<pcl::Normal>::ConstPtr scene_normals_;  ///< Point cloud to be classified
  typename Source<PointT>::ConstPtr m_db_;                ///< model data base
  typename NormalEstimator<PointT>::Ptr
      normal_estimator_;  ///< normal estimator used for computing surface normals (currently only used at training)

  std::string descr_name_;                                  ///< descriptor name
  std::vector<FeatureDescriptor> scene_signatures_;         ///< signatures extracted from the scene
  std::vector<KeypointIndex> keypoint_indices_;             ///< scene point indices extracted as keypoints
  std::vector<KeypointIndex> keypoint_indices_unfiltered_;  ///< only for visualization

  std::vector<LocalObjectModelDatabase::Ptr>
      lomdbs_;  ///< object model database used for local recognition for each feature estiamtor
  std::map<std::string, LocalObjectHypothesis<PointT>>
      corrs_;  ///< correspondences for each object model (model id, correspondences)

  std::vector<typename LocalEstimator<PointT>::Ptr> estimators_;  ///< estimators to compute features/signatures
  std::vector<typename KeypointExtractor<PointT>::Ptr> keypoint_extractor_;  ///< set of keypoint extractors
  std::map<std::string, typename LocalObjectModel::ConstPtr> model_keypoints_;
  PCLVisualizationParams::ConstPtr vis_param_;
  bool visualize_keypoints_;  ///< if true, visualizes the extracted keypoints

  /**
   * @brief since keypoints are coming from multiple local recognizer, we need to store which
   * range belongs to which recognizer. This variable is the starting parting t
   */
  std::vector<std::map<std::string, size_t>> model_kp_idx_range_start_;

  /**
   * @brief this puts the model keypoints extracted from multiple feature estimators into a common database
   */
  void mergeKeypointsFromMultipleEstimators();

  /**
   * @brief validate object
   */
  void validate();

  /**
   * @brief extractKeypoints extracts keypoints from the scene
   * @param[in] region_of_interest object indices (if empty, keypoints will be extracted over whole cloud)
   * @return keypoint indices
   */
  std::vector<KeypointIndex> extractKeypoints(const std::vector<int> &region_of_interest = std::vector<int>());

  /**
   * @brief featureMatching matches all scene keypoints with model signatures
   * @param kp_indices query keypoint indices
   * @param signatures query feature descriptors
   * @param lomdb search space
   */
  void featureMatching(const std::vector<KeypointIndex> &kp_indices, const cv::Mat &signatures,
                       const LocalObjectModelDatabase::ConstPtr &model_keypoints_, size_t model_keypoint_offset = 0);

  /**
   * @brief featureEncoding describes each keypoint with corresponding feature descriptor
   * @param est feature estimator
   * @param keypoint_indices given keypoint indices
   * @param filtered_keypoint_indices extracted keypoint indices after removing nan points for instance
   * @param signatures feature descriptors
   */
  void featureEncoding(LocalEstimator<PointT> &est, const std::vector<KeypointIndex> &keypoint_indices,
                       std::vector<KeypointIndex> &filtered_keypoint_indices, cv::Mat &signatures);

  /**
   * @brief filterKeypoints filters keypoints based on planarity and closeness to depth discontinuity (if according
   * parameters are set)
   * @param[in] input_keypoints keypoints to be filtered
   * @param[inout] input_signatures optional can also filter associated signatures (e.g. for SIFT, keypoint detection
   * and feature description happens in one step - therefore we can only filter after feature description), this
   * variable will update the signatures
   * @return filtered keypoints
   */
  std::vector<int> getInlier(const std::vector<KeypointIndex> &input_keypoints) const;

  void visualizeKeypoints(const std::vector<KeypointIndex> &kp_indices,
                          const std::vector<KeypointIndex> &unfiltered_kp_indices = std::vector<KeypointIndex>()) const;

  /**
   * remove signatures and associated keypoints which contain NaN elements
   * @param signatures
   * @param keypoint_indices
   */
  void removeNaNSignatures(cv::Mat &signatures, std::vector<KeypointIndex> &keypoint_indices) const;

  /**
   * checks if keypoint detector is implicitly integrated in feature estimators
   * @return true if keypoint detector is not integrated in any given feature estimator
   */
  bool needKeypointDetector() const {
    for (const auto &est : estimators_) {
      if (est->detectsKeypoints())
        return false;
    }
    return true;
  }

 public:
  LocalFeatureMatcher(const LocalRecognizerParameter &p = LocalRecognizerParameter())
  : visualize_keypoints_(false), param_(p) {}

  LocalRecognizerParameter param_;  ///< parameters

  /**
   * @brief getFeatureType
   * @return unique feature type id of estimator
   */
  size_t getFeatureType() const {
    size_t type = 0;
    for (const auto &est : estimators_)
      type |= est->getFeatureType();

    return type;
  }

  /**
   * \brief Sets the local feature estimator
   * \param estimator feature estimator
   */
  void addFeatureEstimator(const typename LocalEstimator<PointT>::Ptr &feat) {
    estimators_.push_back(feat);
  }

  /**
   * \brief Initializes the FLANN structure from the provided source
   * It does training for the models that haven't been trained yet
   * @param training directory
   * @param retrain if set to true, re-trains the object no matter if the data already exists in the given training
   * directory
   * @param object_instances_to_load vector of object models to load from model_database_path. If empty, all objects
   * in directory will be loaded.
   */
  void initialize(const bf::path &trained_dir, bool retrain = false,
                  const std::vector<std::string> &object_instances_to_load = {});

  /**
   * @brief adds a keypoint extractor
   * @param keypoint extractor object
   */
  void addKeypointExtractor(typename KeypointExtractor<PointT>::Ptr &ke) {
    keypoint_extractor_.push_back(ke);
  }

  /**
   * @brief needNormals
   * @return boolean indicating if normals need to be set
   */
  bool needNormals() const {
    for (const auto &est : estimators_) {
      if (est && est->needNormals())
        return true;
    }

    if (!keypoint_extractor_.empty()) {
      for (size_t i = 0; i < keypoint_extractor_.size(); i++) {
        if (keypoint_extractor_[i]->needNormals())
          return true;
      }
    }
    return false;
  }

  /**
   * \brief Performs recognition and pose estimation on the input cloud
   */
  void recognize();

  /**
   * @brief getLocalObjectModelDatabase
   * @return local object model database
   */
  typename std::map<std::string, typename LocalObjectModel::ConstPtr> getModelKeypoints() const {
    return model_keypoints_;
  }

  /**
   * @brief getCorrespondences
   * @return all extracted correspondences between keypoints of the local object models and the input cloud
   */
  std::map<std::string, LocalObjectHypothesis<PointT>> getCorrespondences() const {
    return corrs_;
  }

  /**
   * @brief setInputCloud
   * @param cloud to be recognized
   */
  void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr cloud) {
    scene_ = cloud;
  }

  /**
   * @brief setSceneNormals
   * @param normals normals of the input cloud
   */
  void setSceneNormals(const pcl::PointCloud<pcl::Normal>::ConstPtr &normals) {
    scene_normals_ = normals;
  }

  /**
   * @brief setModelDatabase
   * @param m_db model database
   */
  void setModelDatabase(const typename Source<PointT>::ConstPtr &m_db) {
    m_db_ = m_db;
  }

  /**
   * @brief setNormalEstimator sets the normal estimator used for computing surface normals (currently only used at
   * training)
   * @param normal_estimator
   */
  void setNormalEstimator(const typename NormalEstimator<PointT>::Ptr &normal_estimator) {
    normal_estimator_ = normal_estimator;
  }

  /**
   * @brief setVisualizeKeypoints setter function for visualizing keypoints
   * @param vis true if visualization should be turned on
   */
  void setVisualizeKeypoints(bool vis = true) {
    visualize_keypoints_ = vis;
  }

  /**
   * @brief setVisualizationParameter sets the PCL visualization parameter (only used if some visualization is enabled)
   * @param vis_param
   */
  void setVisualizationParameter(const PCLVisualizationParams::ConstPtr &vis_param) {
    vis_param_ = vis_param;
  }

  size_t getNumEstimators() const {
    return estimators_.size();
  }

  typedef std::shared_ptr<LocalFeatureMatcher<PointT>> Ptr;
  typedef std::shared_ptr<LocalFeatureMatcher<PointT> const> ConstPtr;
};
}  // namespace v4r
