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
 * @file ObjectRecognizer.cpp
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#include <iostream>
#include <sstream>

#include <glog/logging.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#include <v4r/apps/ObjectRecognizer.h>
#include <v4r/apps/ViewRenderer.h>

#if HAVE_V4R_CHANGE_DETECTION
#include <v4r/change_detection/change_detection.h>
#include <v4r/change_detection/miscellaneous.h>
#endif

#include <v4r/common/miscellaneous.h>
#include <v4r/common/noise_models.h>
#include <v4r/features/local_estimator_2d.h>
//#include <v4r/features/rops_local_estimator.h>
#include <v4r/features/FeatureDetector_KD_SIFTGPU.h>
#include <v4r/keypoints/all_headers.h>
#include <v4r/ml/all_headers.h>
#include <v4r/recognition/global_recognition_pipeline.h>
#include <v4r/recognition/multiview_recognizer.h>
#include <v4r/registration/noise_model_based_cloud_integration.h>
#include <v4r/segmentation/plane_utils.h>
#include <v4r/segmentation/segmentation_utils.h>

namespace po = boost::program_options;

namespace v4r {

namespace apps {

template <typename PointT>
FeatureDetector::Ptr ObjectRecognizer<PointT>::getFeatureDetector(const v4r::FeatureDetector::Type &type) const {
  switch (type) {
#if HAVE_V4R_FEATURES_SIFT_LOCAL_ESTIMATOR
    case v4r::FeatureDetector::Type::KD_CVSIFT: {
      FeatureDetector_KD_CVSIFT::Ptr feat_det(new v4r::FeatureDetector_KD_CVSIFT);
      return feat_det;
      break;
    }
#endif
#if HAVE_V4R_FEATURES_SIFTGPU_DETECTOR
    case v4r::FeatureDetector::Type::KD_SIFTGPU: {
      FeatureDetector_KD_SIFTGPU::Ptr feat_det(new v4r::FeatureDetector_KD_SIFTGPU);
      return feat_det;
      break;
    }
#endif
#if HAVE_V4R_FEATURES_AKAZE_LOCAL_ESTIMATOR
    case v4r::FeatureDetector::Type::KD_AKAZE: {
      FeatureDetector_KD_AKAZE::Ptr feat_det(new FeatureDetector_KD_AKAZE(param_.feat_params_.akaze_));
      return feat_det;
      break;
    }
#endif
#if HAVE_V4R_FEATURES_OPENCV_XFEATURES2D
    case v4r::FeatureDetector::Type::KD_SURF: {
      FeatureDetector_KD_SURF::Ptr feat_det(new FeatureDetector_KD_SURF(param_.feat_params_.surf_));
      return feat_det;
      break;
    }
    case v4r::FeatureDetector::Type::D_FREAK: {
      FeatureDetector_D_FREAK::Ptr feat_det(new FeatureDetector_D_FREAK(param_.feat_params_.freak_));
      return feat_det;
      break;
    }
#endif
    case v4r::FeatureDetector::Type::KD_BRISK: {
      FeatureDetector_KD_BRISK::Ptr feat_det(new FeatureDetector_KD_BRISK(param_.feat_params_.brisk_));
      return feat_det;
      break;
    }
    case v4r::FeatureDetector::Type::K_HARRIS: {
      FeatureDetector_K_HARRIS::Ptr feat_det(new FeatureDetector_K_HARRIS());
      return feat_det;
      break;
    }
    case v4r::FeatureDetector::Type::K_MSER: {
      FeatureDetector_K_MSER::Ptr feat_det(new FeatureDetector_K_MSER(param_.feat_params_.mser_));
      return feat_det;
      break;
    }
    case v4r::FeatureDetector::Type::KD_FAST_IMGD: {
      FeatureDetector_KD_FAST_IMGD::Ptr feat_det(new FeatureDetector_KD_FAST_IMGD());
      return feat_det;
      break;
    }
    case v4r::FeatureDetector::Type::KD_ORB: {
      FeatureDetector_KD_ORB::Ptr feat_det(new FeatureDetector_KD_ORB());
      return feat_det;
      break;
    }
    default:
      LOG(ERROR) << "Given feature estimator type not implemented!";
      return nullptr;
  }
}

template <typename PointT>
void ObjectRecognizer<PointT>::setupLocal2DPipeLine() {
  typename LocalFeatureMatcher<PointT>::Ptr loc_feat_match(new LocalFeatureMatcher<PointT>(param_.local_2d_pipeline_));
  typename LocalEstimator2D<PointT>::Ptr est_2d(new LocalEstimator2D<PointT>);
  est_2d->setMaxDistance(std::numeric_limits<float>::max());
  est_2d->setKeypointDetector(getFeatureDetector(param_.local_2D_feat_detector_));
  est_2d->setFeatureDetector(getFeatureDetector(param_.local_2D_feat_est_));
  loc_feat_match->addFeatureEstimator(est_2d);
  local_recognition_pipeline_->addLocalFeatureMatcher(loc_feat_match);
}

template <typename PointT>
void ObjectRecognizer<PointT>::initialize(std::vector<std::string> &command_line_arguments,
                                          const bf::path &config_folder) {
  bool retrain = false;
  bool render_training_views_from_mesh_model = false;
  bf::path camera_calibration_file = v4r::io::getConfigDir() / "rgb_calibration.yaml";
  bf::path rgb_depth_overlap_image = v4r::io::getConfigDir() / "rgb_depth_overlap.png";
  const bf::path config_file = "config.ini";
  std::vector<std::string> object_models = {};

  po::options_description desc("Object Instance Recognizer\n======================================\n**Allowed options");
  desc.add_options()("help,h", "produce help message");
  desc.add_options()("model_dir,m", po::value<bf::path>(&models_dir_)->required(), "Directory with object models.");
  desc.add_options()("models", po::value<std::vector<std::string>>(&object_models)->multitoken(),
                     "Names of object models to load from the models directory. If empty or not set, all object models "
                     "will be loaded from the object models directory.");
  desc.add_options()("visualize,v", po::bool_switch(&visualize_), "visualize recognition results");
  desc.add_options()("retrain", po::bool_switch(&retrain),
                     "If set, retrains the object models no matter if they already exists.");
  desc.add_options()("render_views", po::bool_switch(&render_training_views_from_mesh_model),
                     "if enabled, renders training views from a (textured) mesh model");
  desc.add_options()("camera_calibration_file",
                     po::value<bf::path>(&camera_calibration_file)->default_value(camera_calibration_file),
                     "Calibration file of RGB Camera containing intrinsic parameters");
  desc.add_options()(
      "rgb_depth_overlap_image", po::value<bf::path>(&rgb_depth_overlap_image)->default_value(rgb_depth_overlap_image),
      "binary image mask depicting the overlap of the registered RGB and depth image. Pixel values of 255 (white) "
      "indicate that the pixel is visible by both RGB and depth after registration. Black pixel are only seen by the "
      "RGB image due to the physical displacement between RGB and depth sensor.");
  param_.init(desc);

  if (param_.remove_planes_) {
    plane_extractor_.reset(new v4r::apps::CloudSegmenter<PointT>(param_.plane_filter_));
  }

  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(command_line_arguments).options(desc).run();
  std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);

  po::store(parsed, vm);

  bf::path config_path = config_folder / config_file;
  std::ifstream f((config_folder / config_file).string());
  CHECK(v4r::io::existsFile(config_path)) << config_path.string() << " does not exist!";
  po::parsed_options config_parsed = po::parse_config_file(f, desc);
  po::store(config_parsed, vm);
  f.close();

  if (vm.count("help")) {
    std::cout << desc << std::endl;
  }
  try {
    po::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
  }

  if (plane_extractor_)
    plane_extractor_->initialize(to_pass_further);

  try {
    param_.cam_ = Intrinsics::load(camera_calibration_file.string());
  } catch (const std::runtime_error &e) {
    LOG(WARNING) << "Failed to load camera calibration file from " << camera_calibration_file.string()
                 << "! Will use Primesense default camera intrinsics parameters!" << std::endl;
    param_.cam_ = Intrinsics::PrimeSense();
  }

  // ==== RENDER VIEWS FROM OBJECT MODELS (IF ENABLED) =====
  if (render_training_views_from_mesh_model) {
    v4r::apps::ViewRenderer renderer(param_.cam_, param_.rendering_);
    const std::vector<std::string> model_folders = v4r::io::getFoldersInDirectory(models_dir_);
    for (const std::string &m : model_folders) {
      if (std::find(object_models.begin(), object_models.end(), m) != object_models.end()) {
        renderer.render(models_dir_ / bf::path(m) / "tex_mesh.obj", models_dir_ / bf::path(m) / "rendered_views");
      }
    }
  }

  // ==== FILL OBJECT MODEL DATABASE ==== ( assumes each object is in a separate folder named after the object and
  // contains and "views" folder with the training views of the object)
  SourceParameter source_param;
  if (render_training_views_from_mesh_model)
    source_param.view_folder_name_ = "rendered_views";
  model_database_.reset(new Source<PointT>(source_param));
  model_database_->init(models_dir_, object_models);

  normal_estimator_ = v4r::initNormalEstimator<PointT>(param_.normal_computation_method_, to_pass_further);

  // ====== SETUP MULTI PIPELINE RECOGNIZER ======
  typename v4r::MultiRecognitionPipeline<PointT>::Ptr multipipeline(new v4r::MultiRecognitionPipeline<PointT>);
  local_recognition_pipeline_.reset(new LocalRecognitionPipeline<PointT>(param_.local_rec_pipeline_));
  {
    // ====== SETUP LOCAL RECOGNITION PIPELINE =====
    if (param_.do_local_2d_ || param_.do_shot_) {
      local_recognition_pipeline_->setModelDatabase(model_database_);

      if (!param_.use_multiview_ || !param_.use_multiview_with_kp_correspondence_transfer_) {
        if (param_.use_graph_based_gc_grouping_) {
          GraphGeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>::Ptr gc_clusterer(
              new GraphGeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>(param_.gc_));
          local_recognition_pipeline_->setCGAlgorithm(gc_clusterer);
        } else {
          std::shared_ptr<pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>> gc_clusterer(
              new pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>);
          gc_clusterer->setGCSize(param_.gc_.gc_size_);
          gc_clusterer->setGCThreshold(param_.gc_.gc_threshold_);
          local_recognition_pipeline_->setCGAlgorithm(gc_clusterer);
        }
      }

      if (param_.do_local_2d_)
        setupLocal2DPipeLine();

      if (param_.do_shot_) {
        typename LocalFeatureMatcher<PointT>::Ptr shot_rec(new LocalFeatureMatcher<PointT>(param_.shot_pipeline_));
        std::vector<typename v4r::KeypointExtractor<PointT>::Ptr> keypoint_extractor =
            initKeypointExtractors<PointT>(param_.shot_keypoint_extractor_method_, to_pass_further);

        for (typename v4r::KeypointExtractor<PointT>::Ptr ke : keypoint_extractor)
          shot_rec->addKeypointExtractor(ke);

        for (float support_radius : param_.keypoint_support_radii_) {
          SHOTLocalEstimationParameter shot_le_param;
          shot_le_param.support_radius_ = support_radius;

          typename SHOTLocalEstimation<PointT>::Ptr shot_est(new SHOTLocalEstimation<PointT>(shot_le_param));

          //                ROPSLocalEstimationParameter rops_param;
          //                rops_param.init( to_pass_further );
          //                typename ROPSLocalEstimation<PointT>::Ptr rops_est (new ROPSLocalEstimation<PointT>
          //                (rops_param) );

          shot_rec->addFeatureEstimator(shot_est);
        }
        shot_rec->setVisualizeKeypoints(param_.visualize_keypoints_);
        local_recognition_pipeline_->addLocalFeatureMatcher(shot_rec);
      }

      typename RecognitionPipeline<PointT>::Ptr rec_pipeline_tmp =
          std::static_pointer_cast<RecognitionPipeline<PointT>>(local_recognition_pipeline_);
      multipipeline->addRecognitionPipeline(rec_pipeline_tmp);
    }

    // ====== SETUP GLOBAL RECOGNITION PIPELINE =====

    if (!param_.global_feature_types_.empty()) {
      CHECK(param_.global_feature_types_.size() == param_.classification_methods_.size());

      typename GlobalRecognitionPipeline<PointT>::Ptr global_recognition_pipeline(
          new GlobalRecognitionPipeline<PointT>);
      typename v4r::Segmenter<PointT>::Ptr segmenter =
          v4r::initSegmenter<PointT>(param_.segmentation_method_, to_pass_further);
      global_recognition_pipeline->setSegmentationAlgorithm(segmenter);

      for (size_t global_pipeline_id = 0; global_pipeline_id < param_.global_feature_types_.size();
           global_pipeline_id++) {
        GlobalConcatEstimatorParameter p;
        p.feature_type = param_.global_feature_types_[global_pipeline_id];
        typename GlobalConcatEstimator<PointT>::Ptr global_concat_estimator(
            new GlobalConcatEstimator<PointT>(to_pass_further, p));

        //                    typename OURCVFHEstimator<PointT>::Ptr ourcvfh_estimator (new OURCVFHEstimator<PointT>);
        Classifier::Ptr classifier =
            initClassifier(param_.classification_methods_[global_pipeline_id], to_pass_further);

        typename GlobalRecognizer<PointT>::Ptr global_r(new GlobalRecognizer<PointT>(param_.global_rec_));
        global_r->setFeatureEstimator(global_concat_estimator);
        global_r->setClassifier(classifier);
        global_recognition_pipeline->addRecognizer(global_r);
      }

      global_recognition_pipeline->setVisualizeClusters(param_.visualize_global_results_);

      typename RecognitionPipeline<PointT>::Ptr rec_pipeline_tmp =
          std::static_pointer_cast<RecognitionPipeline<PointT>>(global_recognition_pipeline);
      multipipeline->addRecognitionPipeline(rec_pipeline_tmp);
    }

    multipipeline->setModelDatabase(model_database_);
    multipipeline->setNormalEstimator(normal_estimator_);
    multipipeline->setVisualizationParameter(param_.visualization_);
  }

  if (param_.use_multiview_) {
    typename RecognitionPipeline<PointT>::Ptr rec_pipeline =
        std::static_pointer_cast<RecognitionPipeline<PointT>>(multipipeline);
    typename MultiviewRecognizer<PointT>::Ptr mv_rec(new v4r::MultiviewRecognizer<PointT>(param_.multiview_));
    mv_rec->setSingleViewRecognitionPipeline(rec_pipeline);
    mv_rec->setModelDatabase(model_database_);

    if (param_.use_graph_based_gc_grouping_ && param_.multiview_.transfer_keypoint_correspondences_) {
      GraphGeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>::Ptr gc_clusterer(
          new GraphGeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>(param_.gc_));
      mv_rec->setCGAlgorithm(gc_clusterer);
    } else {
      std::shared_ptr<pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>> gc_clusterer(
          new pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>);
      gc_clusterer->setGCSize(param_.gc_.gc_size_);
      gc_clusterer->setGCThreshold(param_.gc_.gc_threshold_);
      mv_rec->setCGAlgorithm(gc_clusterer);
    }

    mrec_ = mv_rec;
  } else
    mrec_ = multipipeline;

  mrec_->initialize(models_dir_, retrain, object_models);

  if (!param_.skip_verification_) {
    hv_.reset(new HypothesisVerification<PointT>(param_.cam_, param_.hv_));

    cv::Mat_<uchar> img_mask = cv::imread(rgb_depth_overlap_image.string(), CV_LOAD_IMAGE_GRAYSCALE);
    if (img_mask.data)
      hv_->setRGBDepthOverlap(img_mask);
    else
      LOG(WARNING) << "No camera depth registration mask provided. Assuming all pixels have valid depth.";

    if (param_.visualize_hv_go_cues_)
      hv_->visualizeCues(param_.visualization_);
    if (param_.visualize_hv_model_cues_)
      hv_->visualizeModelCues(param_.visualization_);
    if (param_.visualize_hv_pairwise_cues_)
      hv_->visualizePairwiseCues(param_.visualization_);

    hv_->setModelDatabase(model_database_);
  }

  if (visualize_) {
    rec_vis_.reset(new v4r::ObjectRecognitionVisualizer<PointT>(param_.vis_layout_));
    rec_vis_->setModelDatabase(model_database_);
  }
}

#if HAVE_V4R_CHANGE_DETECTION
template <typename PointT>
void ObjectRecognizer<PointT>::detectChanges(View &v) {
  v.removed_points_.reset(new pcl::PointCloud<PointT>);

  typename pcl::PointCloud<PointT>::Ptr new_observation_aligned(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*v.processed_cloud_, *new_observation_aligned, v.camera_pose_);

  // downsample
  float resolution = 0.005f;
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(new_observation_aligned);
  vg.setLeafSize(resolution, resolution, resolution);
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  vg.filter(*cloud_filtered);
  new_observation_aligned = cloud_filtered;

  if (registered_scene_cloud_ && !registered_scene_cloud_->points.empty()) {
    v4r::ChangeDetector<PointT> detector;
    detector.detect(registered_scene_cloud_, new_observation_aligned, Eigen::Affine3f(v.camera_pose_),
                    param_.tolerance_for_cloud_diff_);
    //        v4r::ChangeDetector<PointT>::removePointsFrom(registered_scene_cloud_, detector.getRemoved());
    *v.removed_points_ += *(detector.getRemoved());
    //        *changing_scene += *(detector.getAdded());
  }
}
#endif

template <typename PointT>
std::vector<ObjectHypothesesGroup> ObjectRecognizer<PointT>::recognize(
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector<std::string> &obj_models_to_search) {
  // reset view point - otherwise this messes up PCL's visualization (this does not affect recognition results)
  //    cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
  //    cloud->sensor_origin_ = Eigen::Vector4f::Zero(4);

  const Eigen::Matrix4f camera_pose = v4r::RotTrans2Mat4f(cloud->sensor_orientation_, cloud->sensor_origin_);

  typename pcl::PointCloud<PointT>::Ptr processed_cloud(new pcl::PointCloud<PointT>(*cloud));

  std::vector<ObjectHypothesesGroup> generated_object_hypotheses;

  pcl::StopWatch t_total;
  elapsed_time_.clear();

  pcl::PointCloud<pcl::Normal>::Ptr normals;
  if (mrec_->needNormals() || hv_) {
    pcl::StopWatch t;
    const std::string time_desc("Computing normals");
    normal_estimator_->setInputCloud(processed_cloud);
    normals = normal_estimator_->compute();
    mrec_->setSceneNormals(normals);
    double time = t.getTime();
    VLOG(1) << time_desc << " took " << time << " ms.";
    elapsed_time_.push_back(std::pair<std::string, float>(time_desc, time));
  }

  Eigen::Vector4f support_plane;
  if (param_.remove_planes_) {
    pcl::StopWatch t;
    const std::string time_desc("Removing planes");

    plane_extractor_->setNormals(normals);
    plane_extractor_->segment(processed_cloud);
    processed_cloud = plane_extractor_->getProcessedCloud();
    support_plane = plane_extractor_->getSelectedPlane();
    mrec_->setTablePlane(support_plane);

    double time = t.getTime();
    VLOG(1) << time_desc << " took " << time << " ms.";
    elapsed_time_.push_back(std::pair<std::string, float>(time_desc, time));
  }

  // ==== FILTER POINTS BASED ON DISTANCE =====
  for (PointT &p : processed_cloud->points) {
    if (pcl::isFinite(p) && p.getVector3fMap().norm() > param_.chop_z_)
      p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
  }

  {
    pcl::StopWatch t;
    const std::string time_desc("Generation of object hypotheses");

    mrec_->setInputCloud(processed_cloud);
    mrec_->recognize(obj_models_to_search);
    generated_object_hypotheses = mrec_->getObjectHypothesis();

    double time = t.getTime();
    VLOG(1) << time_desc << " took " << time << " ms.";
    elapsed_time_.push_back(std::pair<std::string, float>(time_desc, time));
    std::vector<std::pair<std::string, float>> elapsed_times_rec = mrec_->getElapsedTimes();
    elapsed_time_.insert(elapsed_time_.end(), elapsed_times_rec.begin(), elapsed_times_rec.end());
  }

  //    if(param_.icp_iterations_)
  //    {
  //        refinePose(processed_cloud);
  //    }

  if (param_.skip_verification_ && param_.icp_iterations_) {
    for (size_t ohg_id = 0; ohg_id < generated_object_hypotheses.size(); ohg_id++) {
      for (size_t oh_id = 0; oh_id < generated_object_hypotheses[ohg_id].ohs_.size(); oh_id++) {
        ObjectHypothesis::Ptr &oh = generated_object_hypotheses[ohg_id].ohs_[oh_id];

        bool found_model_foo;
        typename Model<PointT>::ConstPtr m = model_database_->getModelById("", oh->model_id_, found_model_foo);
        typename pcl::PointCloud<PointT>::ConstPtr model_cloud =
            m->getAssembled(5);  // use full resolution for rendering

        const Eigen::Matrix4f hyp_tf_2_global = oh->pose_refinement_ * oh->transform_;
        typename pcl::PointCloud<PointT>::Ptr model_cloud_aligned(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*model_cloud, *model_cloud_aligned, hyp_tf_2_global);

        typename pcl::search::KdTree<PointT>::Ptr kdtree_scene(new pcl::search::KdTree<PointT>);
        kdtree_scene->setInputCloud(processed_cloud);
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(model_cloud_aligned);
        icp.setInputTarget(processed_cloud);
        icp.setTransformationEpsilon(1e-6);
        icp.setMaximumIterations(static_cast<int>(param_.icp_iterations_));
        icp.setMaxCorrespondenceDistance(0.02);
        icp.setSearchMethodTarget(kdtree_scene, true);
        pcl::PointCloud<PointT> aligned_visible_model;
        icp.align(aligned_visible_model);

        Eigen::Matrix4f pose_refinement;
        if (icp.hasConverged()) {
          pose_refinement = icp.getFinalTransformation();
          oh->pose_refinement_ = pose_refinement * oh->pose_refinement_;
        } else
          LOG(WARNING) << "ICP did not converge" << std::endl;
      }
    }
  }

  if (!param_.skip_verification_) {
    hv_->setHypotheses(generated_object_hypotheses);

    if (param_.use_multiview_ && param_.use_multiview_hv_) {
      NMBasedCloudIntegrationParameter nm_int_param;
      nm_int_param.min_points_per_voxel_ = 1;
      nm_int_param.octree_resolution_ = 0.002f;

      NguyenNoiseModelParameter nm_param;

      View v;
      v.cloud_ = cloud;
      v.processed_cloud_ = processed_cloud;
      v.camera_pose_ = camera_pose;
      v.cloud_normals_ = normals;

      {
        pcl::StopWatch t;
        const std::string time_desc("Computing noise model");
        NguyenNoiseModel<PointT> nm(nm_param);
        nm.setInputCloud(processed_cloud);
        nm.setInputNormals(normals);
        nm.compute();
        v.pt_properties_ = nm.getPointProperties();
        double time = t.getTime();
        VLOG(1) << time_desc << " took " << time << " ms.";
        elapsed_time_.push_back(std::pair<std::string, float>(time_desc, time));
      }

      size_t num_views = std::min<size_t>(param_.multiview_max_views_, views_.size() + 1);
      LOG(INFO) << "Running multi-view recognition over " << num_views;

#if HAVE_V4R_CHANGE_DETECTION
      if (param_.use_change_detection_ && !views_.empty()) {
        pcl::StopWatch t;
        const std::string time_desc("Change detection");
        detectChanges(v);

        typename pcl::PointCloud<PointT>::Ptr removed_points_cumulative(
            new pcl::PointCloud<PointT>(*v.removed_points_));

        for (int v_id = (int)views_.size() - 1; v_id >= std::max<int>(0, (int)views_.size() - num_views); v_id--) {
          View &vv = views_[v_id];

          typename pcl::PointCloud<PointT>::Ptr view_aligned(new pcl::PointCloud<PointT>);
          pcl::transformPointCloud(*vv.processed_cloud_, *view_aligned, vv.camera_pose_);

          typename pcl::PointCloud<PointT>::Ptr cloud_tmp(new pcl::PointCloud<PointT>);

          if (vv.removed_points_)
            *removed_points_cumulative += *vv.removed_points_;

          if (!removed_points_cumulative->points.empty()) {
            std::vector<int> preserved_indices;
            v4r::ChangeDetector<PointT>::difference(*view_aligned, removed_points_cumulative, *cloud_tmp,
                                                    preserved_indices, param_.tolerance_for_cloud_diff_);

            /* Visualization of changes removal for reconstruction:
            Cloud rec_changes;
            rec_changes += *cloud_transformed;
            v4r::VisualResultsStorage::copyCloudColored(*removed_points_cumulated_history_[view_id], rec_changes, 255,
            0, 0);
            v4r::VisualResultsStorage::copyCloudColored(*cloud_tmp, rec_changes, 200, 0, 200);
            stringstream ss;
            ss << view_id;
            visResStore.savePcd("reconstruction_changes_" + ss.str(), rec_changes);*/

            boost::dynamic_bitset<> preserved_mask(view_aligned->points.size(), 0);
            for (int idx : preserved_indices)
              preserved_mask.set(idx);

            for (size_t j = 0; j < preserved_mask.size(); j++) {
              if (!preserved_mask[j]) {
                PointT &p = vv.processed_cloud_->points[j];
                p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
              }
            }
            LOG(INFO) << "Points removed in view " << v_id
                      << " by change detection: " << vv.processed_cloud_->points.size() - preserved_indices.size()
                      << ".";
          }
        }

        float time = t.getTime();
        VLOG(1) << time_desc << " took " << time << " ms.";
        elapsed_time_.push_back(std::pair<std::string, float>(time_desc, time));
      }
#else
      if (param_.use_change_detection_ && !views_.empty())
        LOG(ERROR) << "Change detection enabled by parameter settings but change detection module is not available. "
                      "Did you compile it? ";
#endif

      views_.push_back(v);

      std::vector<typename pcl::PointCloud<PointT>::ConstPtr> views(num_views);  ///< all views in multi-view sequence
      std::vector<typename pcl::PointCloud<PointT>::ConstPtr> processed_views(
          num_views);  ///< all processed views in multi-view sequence
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> camera_poses(
          num_views);  ///< all absolute camera poses in multi-view sequence
      std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> views_normals(
          num_views);  ///< all view normals in multi-view sequence
      std::vector<std::vector<std::vector<float>>> views_pt_properties(
          num_views);  ///< all Nguyens noise model point properties in multi-view sequence

      size_t tmp_id = 0;
      for (size_t v_id = views_.size() - num_views; v_id < views_.size(); v_id++) {
        const View &vv = views_[v_id];
        views[tmp_id] = vv.cloud_;
        processed_views[tmp_id] = vv.processed_cloud_;
        camera_poses[tmp_id] = vv.camera_pose_;  // take the current view as the new common reference frame
        views_normals[tmp_id] = vv.cloud_normals_;
        views_pt_properties[tmp_id] = vv.pt_properties_;
        tmp_id++;
      }

      {
        pcl::StopWatch t;
        const std::string time_desc("Noise model based cloud integration");
        registered_scene_cloud_.reset(new pcl::PointCloud<PointT>);
        NMBasedCloudIntegration<PointT> nmIntegration(nm_int_param);
        nmIntegration.setInputClouds(processed_views);
        nmIntegration.setPointProperties(views_pt_properties);
        nmIntegration.setTransformations(camera_poses);
        nmIntegration.setInputNormals(views_normals);
        nmIntegration.compute(registered_scene_cloud_);  // is in global reference frame
        nmIntegration.getOutputNormals(normals);

        double time = t.getTime();
        VLOG(1) << time_desc << " took " << time << " ms.";
        elapsed_time_.push_back(std::pair<std::string, float>(time_desc, time));
      }

      //            static pcl::visualization::PCLVisualizer vis ("final registration");
      //            int vp1, vp2, vp3;
      //            vis.createViewPort(0,0,0.33,1,vp1);
      //            vis.createViewPort(0.33,0,0.66,1,vp2);
      //            vis.createViewPort(0.66,0,1,1,vp3);
      //            vis.removeAllPointClouds();

      //            typename pcl::PointCloud<PointT>::Ptr registered_scene_cloud_aligned_vis(new pcl::PointCloud<PointT>
      //            (*registered_scene_cloud_aligned));
      //            typename pcl::PointCloud<PointT>::Ptr registered_scene_cloud_vis(new pcl::PointCloud<PointT>
      //            (*registered_scene_cloud_));
      //            typename pcl::PointCloud<PointT>::Ptr removed_points_vis(new pcl::PointCloud<PointT>
      //            (*v.processed_cloud_));

      //            registered_scene_cloud_aligned_vis->sensor_origin_ = Eigen::Vector4f::Zero();
      //            registered_scene_cloud_aligned_vis->sensor_orientation_ = Eigen::Quaternionf::Identity();
      //            registered_scene_cloud_vis->sensor_origin_ = Eigen::Vector4f::Zero();
      //            registered_scene_cloud_vis->sensor_orientation_ = Eigen::Quaternionf::Identity();
      //            removed_points_vis->sensor_origin_ = Eigen::Vector4f::Zero();
      //            removed_points_vis->sensor_orientation_ = Eigen::Quaternionf::Identity();

      //            vis.addPointCloud(registered_scene_cloud_aligned_vis, "registered_clouda",vp1);
      //            vis.addPointCloud(registered_scene_cloud_vis, "registered_cloudb",vp2);
      //            vis.addPointCloud(removed_points_vis, "registered_cloudc",vp3);
      //            vis.spin();

      const Eigen::Matrix4f tf_global2cam = camera_pose.inverse();

      typename pcl::PointCloud<PointT>::Ptr registerd_scene_cloud_latest_camera_frame(new pcl::PointCloud<PointT>);
      pcl::transformPointCloud(*registered_scene_cloud_, *registerd_scene_cloud_latest_camera_frame, tf_global2cam);
      pcl::PointCloud<pcl::Normal>::Ptr normals_aligned(new pcl::PointCloud<pcl::Normal>);
      v4r::transformNormals(*normals, *normals_aligned, tf_global2cam);

      hv_->setSceneCloud(registerd_scene_cloud_latest_camera_frame);
      hv_->setNormals(normals_aligned);

      for (Eigen::Matrix4f &tf : camera_poses)  // describe the clouds with respect to the most current view
        tf = camera_pose.inverse() * tf;

      hv_->setOcclusionCloudsAndAbsoluteCameraPoses(views, camera_poses);
    } else {
      hv_->setSceneCloud(cloud);
      hv_->setNormals(normals);
    }

    pcl::StopWatch t;
    const std::string time_desc("Verification of object hypotheses");
    hv_->verify();
    double time = t.getTime();
    VLOG(1) << time_desc << " took " << time << " ms.";
    elapsed_time_.push_back(std::pair<std::string, float>(time_desc, time));

    std::vector<std::pair<std::string, float>> hv_elapsed_times = hv_->getElapsedTimes();
    elapsed_time_.insert(elapsed_time_.end(), hv_elapsed_times.begin(), hv_elapsed_times.end());
  }

  if (param_.remove_planes_ && param_.remove_non_upright_objects_) {
    for (size_t ohg_id = 0; ohg_id < generated_object_hypotheses.size(); ohg_id++) {
      for (size_t oh_id = 0; oh_id < generated_object_hypotheses[ohg_id].ohs_.size(); oh_id++) {
        ObjectHypothesis::Ptr &oh = generated_object_hypotheses[ohg_id].ohs_[oh_id];

        if (!oh->is_verified_)
          continue;

        const Eigen::Matrix4f tf = oh->pose_refinement_ * oh->transform_;
        const Eigen::Vector3f translation = tf.block<3, 1>(0, 3);
        double dist2supportPlane = fabs(v4r::dist2plane(translation, support_plane));
        const Eigen::Vector3f z_orientation = tf.block<3, 3>(0, 0) * Eigen::Vector3f::UnitZ();
        float dotp = z_orientation.dot(support_plane.head(3)) / (support_plane.head(3).norm() * z_orientation.norm());
        VLOG(1) << "dotp for model " << oh->model_id_ << ": " << dotp;

        if (dotp < 0.8f) {
          oh->is_verified_ = false;
          VLOG(1) << "Rejected " << oh->model_id_ << " because it is not standing upgright (dot-product = " << dotp
                  << ")!";
        }
        if (dist2supportPlane > 0.03) {
          oh->is_verified_ = false;
          VLOG(1) << "Rejected " << oh->model_id_
                  << " because object origin is too far away from support plane = " << dist2supportPlane << ")!";
        }
      }
    }
  }

  double time_total = t_total.getTime();

  std::stringstream info;
  size_t num_detected = 0;
  for (size_t ohg_id = 0; ohg_id < generated_object_hypotheses.size(); ohg_id++) {
    for (const ObjectHypothesis::Ptr &oh : generated_object_hypotheses[ohg_id].ohs_) {
      if (oh->is_verified_) {
        num_detected++;
        const std::string &model_id = oh->model_id_;
        const Eigen::Matrix4f &tf = oh->transform_;
        float confidence = oh->confidence_;
        info << "" << model_id << " (confidence: " << std::fixed << std::setprecision(2) << confidence
             << ") with pose:" << std::endl
             << std::setprecision(5) << tf << std::endl
             << std::endl;
      }
    }
  }

  if (num_detected) {
    std::stringstream rec_info;
    rec_info << "Detected " << num_detected << " object(s) in " << time_total << "ms" << std::endl << info.str();
    LOG(INFO) << rec_info.str();

    if (!FLAGS_logtostderr)
      std::cout << rec_info.str();
  }

  if (visualize_) {
    const std::map<std::string, typename LocalObjectModel::ConstPtr> lomdb =
        local_recognition_pipeline_->getLocalObjectModelDatabase();
    rec_vis_->setCloud(cloud);

    if (param_.use_multiview_ && param_.use_multiview_hv_ && !param_.skip_verification_) {
      const Eigen::Matrix4f tf_global2camera = camera_pose.inverse();
      typename pcl::PointCloud<PointT>::Ptr registered_scene_cloud_aligned(new pcl::PointCloud<PointT>);
      pcl::transformPointCloud(*registered_scene_cloud_, *registered_scene_cloud_aligned, tf_global2camera);
      rec_vis_->setProcessedCloud(registered_scene_cloud_aligned);
    } else
      rec_vis_->setProcessedCloud(processed_cloud);

    rec_vis_->setNormals(normals);

    rec_vis_->setGeneratedObjectHypotheses(generated_object_hypotheses);
    //        rec_vis_->setRefinedGeneratedObjectHypotheses( generated_object_hypotheses_refined_ );
    rec_vis_->setLocalModelDatabase(lomdb);
    //        rec_vis_->setVerifiedObjectHypotheses( verified_hypotheses_ );
    rec_vis_->visualize();
  }

  return generated_object_hypotheses;
}

template <typename PointT>
void ObjectRecognizer<PointT>::resetMultiView() {
  if (param_.use_multiview_) {
    views_.clear();

    typename v4r::MultiviewRecognizer<PointT>::Ptr mv_rec =
        std::dynamic_pointer_cast<v4r::MultiviewRecognizer<PointT>>(mrec_);
    if (mrec_)
      mv_rec->clear();
    else
      LOG(ERROR) << "Cannot reset multi-view recognizer because given recognizer is not a multi-view recognizer!";
  }
}

template <typename PointT>
typename pcl::PointCloud<PointT>::ConstPtr ObjectRecognizer<PointT>::getModel(const std::string &model_name,
                                                                              int resolution_mm) const {
  bool found;
  typename Source<PointT>::ConstPtr mdb = mrec_->getModelDatabase();
  typename Model<PointT>::ConstPtr model = mdb->getModelById("", model_name, found);
  if (!found) {
    LOG(ERROR) << "Could not find model with name " << model_name;
    return nullptr;
  }

  return model->getAssembled(resolution_mm);
}

template class V4R_EXPORTS ObjectRecognizer<pcl::PointXYZRGB>;
}  // namespace apps
}  // namespace v4r
