#include <glog/logging.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/features/boundary.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_1_8/features/organized_edge_detection.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/io/cv.h>
#include <v4r/io/eigen.h>
#include <v4r/io/filesystem.h>
#include <v4r/recognition/local_feature_matching.h>

#include <opencv2/opencv.hpp>

#include <omp.h>

namespace po = boost::program_options;

namespace v4r {

template <typename PointT>
void LocalFeatureMatcher<PointT>::validate() {
  CHECK(estimators_.size() <= 1 || needKeypointDetector())
      << "Given feature estimator is not allowed to be mixed with other feature descriptors.";

  CHECK(needKeypointDetector() || param_.train_on_individual_views_)
      << "Feature estimator needs organized point clouds. Therefore training from a full model is not supported! "
      << std::endl;
}

template <typename PointT>
void LocalFeatureMatcher<PointT>::visualizeKeypoints(const std::vector<KeypointIndex> &kp_indices,
                                                     const std::vector<KeypointIndex> &unfiltered_kp_indices) const {
  static pcl::visualization::PCLVisualizer::Ptr vis;

  if (!vis)
    vis.reset(new pcl::visualization::PCLVisualizer("keypoints"));

  std::stringstream title;
  title << kp_indices.size() << " keypoints";
  vis->setWindowName(title.str());
  vis->removeAllPointClouds();
  vis->removeAllShapes();
  vis->addPointCloud(scene_, "scene");

  if (!kp_indices.empty()) {
    pcl::PointCloud<PointT> colored_kps;
    pcl::PointCloud<pcl::Normal> kp_normals;
    pcl::PointCloud<PointT> colored_kps_unfiltered;
    pcl::PointCloud<pcl::Normal> kp_unfiltered_normals;
    pcl::copyPointCloud(*scene_, kp_indices, colored_kps);
    pcl::copyPointCloud(*scene_normals_, kp_indices, kp_normals);
    pcl::copyPointCloud(*scene_, unfiltered_kp_indices, colored_kps_unfiltered);
    pcl::copyPointCloud(*scene_normals_, unfiltered_kp_indices, kp_unfiltered_normals);
    for (PointT &p : colored_kps.points) {
      p.r = 255.f;
      p.g = 0.f;
      p.b = 0.f;
    }
    for (PointT &p : colored_kps_unfiltered.points) {
      p.r = 0.f;
      p.g = 255.f;
      p.b = 0.f;
    }

    //        vis_->addPointCloudNormals<PointT, pcl::Normal>(colored_kps_unfiltered.makeShared(),
    //        kp_unfiltered_normals.makeShared(), 10, 0.05, "kp_normals_unfiltered");
    vis->addPointCloud(colored_kps_unfiltered.makeShared(), "kps_unfiltered");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "kps_unfiltered");

    //        vis_->addPointCloudNormals<PointT, pcl::Normal>(colored_kps.makeShared(), kp_normals.makeShared(), 10,
    //        0.05, "normals_model");
    vis->addPointCloud(colored_kps.makeShared(), "kps");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "kps");
  }
  std::stringstream txt_kp;
  txt_kp << "Filtered keypoints (" << kp_indices.size() << ")";
  std::stringstream txt_kp_rejected;
  txt_kp_rejected << "Rejected keypoints (" << kp_indices.size() << ")";

  if (!vis_param_->no_text_) {
    vis->addText(txt_kp.str(), 10, 10, 12, 1., 0, 0, "filtered keypoints");
    vis->addText(txt_kp_rejected.str(), 10, 20, 12, 0, 1., 0, "rejected keypoints");
  }

  vis->setBackgroundColor(1, 1, 1);
  vis->resetCamera();
  vis->spin();
}

template <typename PointT>
std::vector<int> LocalFeatureMatcher<PointT>::getInlier(const std::vector<KeypointIndex> &input_keypoints) const {
  if (input_keypoints.empty())
    return std::vector<int>();

  boost::dynamic_bitset<> kp_is_kept(input_keypoints.size());
  kp_is_kept.set();

  //    if(visualize_keypoints_)
  //        keypoint_indices_unfiltered_ = input_keypoints;

  if (param_.filter_planar_) {
    pcl::StopWatch t;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::NormalEstimationOMP<PointT, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(scene_);
    boost::shared_ptr<std::vector<int>> IndicesPtr(new std::vector<int>);
    *IndicesPtr = input_keypoints;
    normalEstimation.setIndices(IndicesPtr);
    normalEstimation.setRadiusSearch(param_.planar_support_radius_);
    normalEstimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals_for_planarity_check(new pcl::PointCloud<pcl::Normal>);
    normalEstimation.compute(*normals_for_planarity_check);

    for (size_t i = 0; i < input_keypoints.size(); i++) {
      if (normals_for_planarity_check->points[i].curvature < param_.threshold_planar_)
        kp_is_kept.reset(i);
    }

    VLOG(1) << "Filtering planar keypoints took " << t.getTime() << " ms.";
  }

  if (param_.filter_border_pts_) {
    if (scene_->isOrganized()) {
      pcl::StopWatch t;
      // compute depth discontinuity edges
      pcl_1_8::OrganizedEdgeBase<PointT, pcl::Label> oed;
      oed.setDepthDisconThreshold(0.05f);  // at 1m, adapted linearly with depth
      oed.setMaxSearchNeighbors(100);
      oed.setEdgeType(pcl_1_8::OrganizedEdgeBase<PointT, pcl::Label>::EDGELABEL_OCCLUDING |
                      pcl_1_8::OrganizedEdgeBase<PointT, pcl::Label>::EDGELABEL_OCCLUDED |
                      pcl_1_8::OrganizedEdgeBase<PointT, pcl::Label>::EDGELABEL_NAN_BOUNDARY);
      oed.setInputCloud(scene_);

      pcl::PointCloud<pcl::Label> labels;
      std::vector<pcl::PointIndices> edge_indices;
      oed.compute(labels, edge_indices);

      // count indices to allocate memory beforehand
      size_t kept = 0;
      for (size_t j = 0; j < edge_indices.size(); j++)
        kept += edge_indices[j].indices.size();

      std::vector<int> discontinuity_edges(kept);

      kept = 0;
      for (size_t j = 0; j < edge_indices.size(); j++) {
        for (size_t i = 0; i < edge_indices[j].indices.size(); i++)
          discontinuity_edges[kept++] = edge_indices[j].indices[i];
      }

      cv::Mat boundary_mask = cv::Mat_<unsigned char>::zeros(scene_->height, scene_->width);
      for (size_t i = 0; i < discontinuity_edges.size(); i++) {
        int idx = discontinuity_edges[i];
        int u = idx % scene_->width;
        int v = idx / scene_->width;

        boundary_mask.at<unsigned char>(v, u) = 255;
      }

      cv::Mat element = cv::getStructuringElement(
          cv::MORPH_ELLIPSE, cv::Size(2 * param_.boundary_width_ + 1, 2 * param_.boundary_width_ + 1),
          cv::Point(param_.boundary_width_, param_.boundary_width_));
      cv::Mat boundary_mask_dilated;
      cv::dilate(boundary_mask, boundary_mask_dilated, element);

      for (size_t i = 0; i < input_keypoints.size(); i++) {
        int idx = input_keypoints[i];
        int u = idx % scene_->width;
        int v = idx / scene_->width;

        if (boundary_mask_dilated.at<unsigned char>(v, u))
          kp_is_kept.reset(i);
      }
      VLOG(1) << "Computing boundary points took " << t.getTime() << " ms.";
    } else
      LOG(ERROR) << "Input scene is not organized so cannot extract edge points.";
  }

  return createIndicesFromMask<int>(kp_is_kept);
}

template <typename PointT>
std::vector<int> LocalFeatureMatcher<PointT>::extractKeypoints(const std::vector<int> &region_of_interest) {
  if (keypoint_extractor_.empty()) {
    LOG(INFO) << "No keypoint extractor given. Using all points as point of interest.";
    return std::vector<int>();
  }

  pcl::StopWatch t;
  boost::dynamic_bitset<> obj_mask;
  boost::dynamic_bitset<> kp_mask(scene_->points.size(), 0);

  if (region_of_interest.empty())  // if empty take whole cloud
  {
    obj_mask.resize(scene_->points.size(), 0);
    obj_mask.set();
  } else
    obj_mask = createMaskFromIndices(region_of_interest, scene_->points.size());

  bool estimator_need_normals = false;
  for (const typename LocalEstimator<PointT>::ConstPtr &est : estimators_) {
    if (est->needNormals()) {
      estimator_need_normals = true;
      break;
    }
  }

  for (typename KeypointExtractor<PointT>::Ptr ke : keypoint_extractor_) {
    ke->setInputCloud(scene_);
    ke->setNormals(scene_normals_);
    ke->compute();

    const std::vector<int> kp_indices = ke->getKeypointIndices();

    // only keep keypoints which are finite (with finite normals), are closer than the maximum allowed distance,
    // belong to the Region of Interest and are not planar (if planarity filter is on)
    for (int idx : kp_indices) {
      if (obj_mask[idx] && pcl::isFinite(scene_->points[idx]) &&
          (!estimator_need_normals || pcl::isFinite(scene_normals_->points[idx])) &&
          scene_->points[idx].getVector3fMap().norm() < param_.max_keypoint_distance_z_) {
        kp_mask.set(idx);
      }
    }
  }

  VLOG(1) << "Extracting all keypoints with filtering took " << t.getTime() << " ms.";

  return createIndicesFromMask<int>(kp_mask);
}

template <typename PointT>
void LocalFeatureMatcher<PointT>::initialize(const bf::path &trained_dir, bool retrain,
                                             const std::vector<std::string> &object_instances_to_load) {
  CHECK(m_db_);
  validate();
  lomdbs_.resize(estimators_.size());

  std::vector<typename Model<PointT>::ConstPtr> models = m_db_->getModels();

  for (size_t est_id = 0; est_id < estimators_.size(); est_id++) {
    LocalObjectModelDatabase::Ptr lomdb(new LocalObjectModelDatabase);
    cv::Mat all_signatures_;  ///< all signatures from all objects in the database

    typename LocalEstimator<PointT>::Ptr &est = estimators_[est_id];

    for (typename Model<PointT>::ConstPtr m : models) {
      bf::path trained_path_feat =
          trained_dir / m->id_ / bf::path(est->getFeatureDescriptorName() + est->getUniqueId());

      if (!object_instances_to_load.empty() &&
          std::find(object_instances_to_load.begin(), object_instances_to_load.end(), m->id_) ==
              object_instances_to_load.end()) {
        LOG(INFO) << "Skipping object " << m->id_ << " because it is not in the lists of objects to load.";
        continue;
      }

      cv::Mat model_signatures;
      pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::Normal>::Ptr model_kp_normals(new pcl::PointCloud<pcl::Normal>);

      const bf::path kp_path = trained_path_feat / "keypoints.pcd";
      const bf::path kp_normals_path = trained_path_feat / "keypoint_normals.pcd";
      const bf::path signatures_path = trained_path_feat / "signatures.dat";

      if (!retrain && io::existsFile(kp_path) && io::existsFile(kp_normals_path) && io::existsFile(signatures_path)) {
        // load trained models (keypoints and signatures) from disk
        pcl::io::loadPCDFile(kp_path.string(), *model_keypoints);
        pcl::io::loadPCDFile(kp_normals_path.string(), *model_kp_normals);
        model_signatures = io::readMatBinary(signatures_path);
      } else {  // train object model from training data
        const auto training_views = m->getTrainingViews();
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> existing_poses;

        if (param_.train_on_individual_views_) {
          for (const auto &tv : training_views) {
            pcl::StopWatch t;

            std::vector<int> obj_indices;

            Eigen::Matrix4f pose;
            if (tv->cloud_)  // point cloud and all relevant information is already in memory (fast but needs a much
                             // memory when a lot of training views/objects)
            {
              scene_ = tv->cloud_;
              scene_normals_ = tv->normals_;
              obj_indices = tv->indices_;
              pose = tv->pose_;
            } else {
              typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
              pcl::io::loadPCDFile(tv->filename_.string(), *cloud);

              try {
                pose = io::readMatrixFromFile(tv->pose_filename_);
              } catch (const std::runtime_error &e) {
                LOG(ERROR) << "Could not read pose from file " << tv->pose_filename_ << "! Setting it to identity";
                pose = Eigen::Matrix4f::Identity();
              }

              // read object mask from file
              obj_indices.clear();
              if (!io::existsFile(tv->indices_filename_)) {
                LOG(WARNING) << "No object indices " << tv->indices_filename_ << " found for object " << m->class_
                             << "/" << m->id_ << " / " << tv->filename_
                             << "! Taking whole cloud as object of interest!";
              } else {
                std::ifstream mi_f(tv->indices_filename_.string());
                int idx;
                while (mi_f >> idx)
                  obj_indices.push_back(idx);
                mi_f.close();

                boost::dynamic_bitset<> obj_mask = createMaskFromIndices(obj_indices, cloud->points.size());
                for (size_t px = 0; px < cloud->points.size(); px++) {
                  if (!obj_mask[px]) {
                    PointT &p = cloud->points[px];
                    p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
                  }
                }
              }

              scene_ = cloud;

              if (true)  // always needs normals since we never know if correspondence grouping does! .....
                         // this->needNormals() )
              {
                normal_estimator_->setInputCloud(cloud);
                pcl::PointCloud<pcl::Normal>::Ptr normals;
                normals = normal_estimator_->compute();
                scene_normals_ = normals;
              }
            }

            bool similar_pose_exists = false;
            for (const Eigen::Matrix4f &ep : existing_poses) {
              Eigen::Vector3f v1 = pose.block<3, 1>(0, 0);
              Eigen::Vector3f v2 = ep.block<3, 1>(0, 0);
              v1.normalize();
              v2.normalize();
              float dotp = v1.dot(v2);
              const Eigen::Vector3f crossp = v1.cross(v2);

              float rel_angle_deg = acos(dotp) * 180.f / M_PI;
              if (crossp(2) < 0)
                rel_angle_deg = 360.f - rel_angle_deg;

              if (rel_angle_deg < param_.required_viewpoint_change_deg_) {
                similar_pose_exists = true;
                break;
              }
            }
            if (!similar_pose_exists) {
              std::vector<int> filtered_kp_indices;

              if (est->detectsKeypoints())  // for some feature descriptor we do not need to extract keypoints
                                            // explicitly
                filtered_kp_indices = obj_indices;
              else {
                const std::vector<KeypointIndex> keypoint_indices = extractKeypoints(obj_indices);
                std::vector<int> inlier = getInlier(keypoint_indices);
                filtered_kp_indices = filterVector<KeypointIndex>(keypoint_indices, inlier);

                if (visualize_keypoints_)
                  visualizeKeypoints(filtered_kp_indices, keypoint_indices);
              }

              cv::Mat signatures_view;
              featureEncoding(*est, filtered_kp_indices, filtered_kp_indices, signatures_view);

              if (est->detectsKeypoints()) {  // for SIFT we do not need to extract keypoints explicitly
                const std::vector<int> inlier = getInlier(filtered_kp_indices);
                filtered_kp_indices = filterVector<KeypointIndex>(filtered_kp_indices, inlier);
                signatures_view = filterCvMat(signatures_view, inlier);
              }

              if (filtered_kp_indices.empty())
                continue;

              existing_poses.push_back(pose);
              LOG(INFO) << "Adding " << signatures_view.rows << " " << est->getFeatureDescriptorName() << " (with id \""
                        << est->getUniqueId() << "\") descriptors to the model database. " << std::endl;

              CHECK(signatures_view.rows == (int)filtered_kp_indices.size());

              pcl::PointCloud<pcl::PointXYZ> model_keypoints_tmp;
              pcl::PointCloud<pcl::Normal> model_keypoint_normals_tmp;
              pcl::copyPointCloud(*scene_, filtered_kp_indices, model_keypoints_tmp);
              pcl::copyPointCloud(*scene_normals_, filtered_kp_indices, model_keypoint_normals_tmp);
              pcl::transformPointCloud(model_keypoints_tmp, model_keypoints_tmp, pose);
              v4r::transformNormals(model_keypoint_normals_tmp, model_keypoint_normals_tmp, pose);
              *model_keypoints += model_keypoints_tmp;
              *model_kp_normals += model_keypoint_normals_tmp;

              if (model_signatures.empty())
                model_signatures = signatures_view;
              else
                cv::vconcat(model_signatures, signatures_view, model_signatures);

              indices_.clear();
            } else
              LOG(INFO) << "Ignoring view " << tv->filename_ << " because a similar camera pose exists.";

            LOG(INFO) << "Training " << est->getFeatureDescriptorName() << " (with id " << est->getUniqueId()
                      << ") on view " << m->class_ << "/" << m->id_ << "/" << tv->filename_ << ") took " << t.getTime()
                      << " ms.";
          }
        } else {
          scene_ = m->getAssembled(1);
          scene_normals_ = m->getNormalsAssembled(1);

          const std::vector<KeypointIndex> keypoint_indices = extractKeypoints();
          std::vector<int> inlier = getInlier(keypoint_indices);
          std::vector<KeypointIndex> filtered_kp_indices = filterVector<KeypointIndex>(keypoint_indices, inlier);

          if (visualize_keypoints_)
            visualizeKeypoints(filtered_kp_indices, keypoint_indices);

          cv::Mat signatures;
          featureEncoding(*est, filtered_kp_indices, filtered_kp_indices, signatures);

          if (filtered_kp_indices.empty())
            continue;

          LOG(INFO) << "Adding " << signatures.size() << " " << est->getFeatureDescriptorName() << " (with id \""
                    << est->getUniqueId() << "\") descriptors to the model database. ";

          CHECK(signatures.rows == (int)filtered_kp_indices.size());

          pcl::PointCloud<pcl::PointXYZ> model_keypoints_tmp;
          pcl::PointCloud<pcl::Normal> model_keypoint_normals_tmp;
          pcl::copyPointCloud(*scene_, filtered_kp_indices, model_keypoints_tmp);
          pcl::copyPointCloud(*scene_normals_, filtered_kp_indices, model_keypoint_normals_tmp);
          *model_keypoints += model_keypoints_tmp;
          *model_kp_normals += model_keypoint_normals_tmp;
          if (model_signatures.empty())
            model_signatures = signatures;
          else
            cv::vconcat(model_signatures, signatures, model_signatures);
        }

        // save keypoints and signatures to disk
        io::createDirForFileIfNotExist(kp_path.string());
        pcl::io::savePCDFileBinaryCompressed(kp_path.string(), *model_keypoints);
        pcl::io::savePCDFileBinaryCompressed(kp_normals_path.string(), *model_kp_normals);
        io::writeMatBinary(signatures_path, model_signatures);
      }

      if (all_signatures_.empty())
        all_signatures_ = model_signatures;
      else
        cv::vconcat(all_signatures_, model_signatures, all_signatures_);

      std::vector<LocalObjectModelDatabase::flann_model> flann_models_tmp(model_signatures.rows);
      for (int f = 0; f < model_signatures.rows; f++) {
        flann_models_tmp[f].model_id_ = m->id_;
        flann_models_tmp[f].keypoint_id_ = f;
      }
      lomdb->flann_models_.insert(lomdb->flann_models_.end(), flann_models_tmp.begin(), flann_models_tmp.end());

      LocalObjectModel::Ptr lom(new LocalObjectModel);
      lom->keypoints_ = model_keypoints;
      lom->kp_normals_ = model_kp_normals;
      lomdb->l_obj_models_[m->id_] = lom;
    }

    CHECK((int)lomdb->flann_models_.size() == all_signatures_.rows);

    LOG(INFO) << "Building the kdtree index for " << est->getFeatureDescriptorName() << " (with id "
              << est->getUniqueId() << ") for " << all_signatures_.rows << " elements.";

    if (param_.use_brute_force_matching_) {
      switch (param_.distance_metric_) {
        case DistanceMetric::L1:
          lomdb->matcher_.reset(new cv::BFMatcher(cv::NORM_L1, param_.cross_check_));
          break;
        case DistanceMetric::L2:
          lomdb->matcher_.reset(new cv::BFMatcher(cv::NORM_L2, param_.cross_check_));
          break;
        case DistanceMetric::HAMMING:
          lomdb->matcher_.reset(new cv::BFMatcher(cv::NORM_HAMMING, param_.cross_check_));
          break;
        default:
          LOG(ERROR) << "Distance metric " << param_.distance_metric_
                     << " is not implemented for brute force matching!";
      }
    } else {
      switch (param_.distance_metric_) {
        case DistanceMetric::HAMMING:
          lomdb->matcher_.reset(new cv::FlannBasedMatcher(
              new cv::flann::LshIndexParams(param_.lsh_index_table_number_, param_.lsh_index_key_index_,
                                            param_.lsh_index_multi_probe_level_),
              new cv::flann::SearchParams(param_.kdtree_search_checks_, 0, true)));
          break;
        default:
          lomdb->matcher_.reset(
              new cv::FlannBasedMatcher(new cv::flann::KDTreeIndexParams(param_.kdtree_num_trees_),
                                        new cv::flann::SearchParams(param_.kdtree_search_checks_, 0, true)));
      }
    }
    std::vector<cv::Mat> descriptors;
    descriptors.push_back(all_signatures_);
    lomdb->matcher_->add(descriptors);
    lomdb->matcher_->train();

    lomdbs_[est_id] = lomdb;

    VLOG(2) << "Initialized local recognition pipeline - Size of matcher object " << sizeof(*lomdb->matcher_)
            << "bytes, size of flann models: " << sizeof(lomdb->flann_models_) << " bytes.";
  }

  mergeKeypointsFromMultipleEstimators();

  indices_.clear();
}

template <typename PointT>
void LocalFeatureMatcher<PointT>::mergeKeypointsFromMultipleEstimators() {
  model_keypoints_.clear();
  model_kp_idx_range_start_.resize(estimators_.size());

  for (size_t est_id = 0; est_id < estimators_.size(); est_id++) {
    LocalObjectModelDatabase::ConstPtr lomdb_tmp = lomdbs_[est_id];

    for (const auto &lo : lomdb_tmp->l_obj_models_) {
      const std::string &model_id = lo.first;
      const LocalObjectModel &lom = *(lo.second);

      std::map<std::string, typename LocalObjectModel::ConstPtr>::const_iterator it_loh =
          model_keypoints_.find(model_id);

      if (it_loh != model_keypoints_.end()) {  // append keypoints to existing ones
        model_kp_idx_range_start_[est_id][model_id] = it_loh->second->keypoints_->points.size();
        *(it_loh->second->keypoints_) += *(lom.keypoints_);
        *(it_loh->second->kp_normals_) += *(lom.kp_normals_);
      } else {  // keypoints do not exist yet for this model
        model_kp_idx_range_start_[est_id][model_id] = 0;
        LocalObjectModel::Ptr lom_copy(new LocalObjectModel);
        *(lom_copy->keypoints_) = *(lom.keypoints_);
        *(lom_copy->kp_normals_) = *(lom.kp_normals_);
        model_keypoints_[model_id] = lom_copy;
      }
    }
  }
}

template <typename PointT>
void LocalFeatureMatcher<PointT>::featureMatching(const std::vector<KeypointIndex> &kp_indices,
                                                  const cv::Mat &signatures,
                                                  const LocalObjectModelDatabase::ConstPtr &lomdb,
                                                  size_t model_keypoint_offset) {
  CHECK(signatures.rows == (int)kp_indices.size());

  cv::Mat indices;
  cv::Mat distances;
  std::vector<std::vector<cv::DMatch>> matches;
  lomdb->matcher_->knnMatch(signatures, matches, param_.knn_);

  for (const auto &mm : matches) {
    for (const cv::DMatch &m : mm) {
      // if (m.distance > param_.max_descriptor_distance_){
      //  VLOG(0) << "m.distance: " << m.distance << ", param max dist: " << param_.max_descriptor_distance_;
      //  continue;
      //}

      const typename LocalObjectModelDatabase::flann_model &f = lomdb->flann_models_[m.trainIdx];
      float m_dist = param_.correspondence_distance_weight_;  // * m.distance;

      // typename std::map<std::string, LocalObjectModel::ConstPtr>::const_iterator it =
      //    model_keypoints_.find(f.model_id_);
      //            const LocalObjectModel &m_kps = *(it->second);

      KeypointIndex m_idx = f.keypoint_id_ + model_keypoint_offset;
      KeypointIndex s_idx = kp_indices[m.queryIdx];
      //            CHECK ( m_idx < m_kps.keypoints_->points.size() );
      //            CHECK ( kp_indices[idx] < scene_->points.size() );

      typename std::map<std::string, LocalObjectHypothesis<PointT>>::iterator it_c = corrs_.find(f.model_id_);
      if (it_c != corrs_.end()) {  // append correspondences to existing ones
        pcl::CorrespondencesPtr &corrs = it_c->second.model_scene_corresp_;
        corrs->push_back(pcl::Correspondence(m_idx, s_idx, m_dist));
      } else  // create object hypothesis
      {
        LocalObjectHypothesis<PointT> new_loh;
        new_loh.model_scene_corresp_.reset(new pcl::Correspondences);
        new_loh.model_scene_corresp_->push_back(pcl::Correspondence(m_idx, s_idx, m_dist));
        new_loh.model_id_ = f.model_id_;
        corrs_[f.model_id_] = new_loh;
      }
    }
  }
}

template <typename PointT>
void LocalFeatureMatcher<PointT>::removeNaNSignatures(cv::Mat &signatures,
                                                      std::vector<KeypointIndex> &keypoint_indices) const {
  size_t kept = 0;
  for (int sig_id = 0; sig_id < signatures.rows; sig_id++) {
    cv::Mat mask = cv::Mat(signatures.row(sig_id) != signatures.row(sig_id));
    if (!cv::countNonZero(mask)) {
      signatures.row(sig_id).copyTo(signatures.row(kept));
      keypoint_indices[kept] = keypoint_indices[sig_id];
      kept++;
    }
  }
  keypoint_indices.resize(kept);
  signatures.resize(kept);
}

template <typename PointT>
void LocalFeatureMatcher<PointT>::featureEncoding(LocalEstimator<PointT> &est,
                                                  const std::vector<KeypointIndex> &keypoint_indices,
                                                  std::vector<KeypointIndex> &filtered_keypoint_indices,
                                                  cv::Mat &signatures) {
  {
    pcl::StopWatch t;
    est.setInputCloud(scene_);
    est.setNormals(scene_normals_);
    est.setIndices(keypoint_indices);
    est.compute(signatures);
    filtered_keypoint_indices = est.getKeypointIndices();

    CHECK((int)filtered_keypoint_indices.size() == signatures.rows);

    VLOG(1) << "Feature encoding for " << est.getFeatureDescriptorName() << " (with id " << est.getUniqueId()
            << ") took " << t.getTime() << " ms.";
  }
  removeNaNSignatures(signatures, filtered_keypoint_indices);
}

template <typename PointT>
void LocalFeatureMatcher<PointT>::recognize() {
  pcl::StopWatch t_total;
  corrs_.clear();
  keypoint_indices_.clear();

  const std::vector<KeypointIndex> keypoint_indices = extractKeypoints(indices_);
  std::vector<KeypointIndex> filtered_kp_indices;

  if (needKeypointDetector()) {
    std::vector<int> inlier = getInlier(keypoint_indices);
    filtered_kp_indices = filterVector<KeypointIndex>(keypoint_indices, inlier);
  }
  std::string feature_descriptor_name = "";
  for (size_t est_id = 0; est_id < estimators_.size(); est_id++) {
    typename LocalEstimator<PointT>::Ptr &est = estimators_[est_id];
    feature_descriptor_name = estimators_[est_id]->getFeatureDescriptorName();
    std::vector<KeypointIndex> filtered_kp_indices_tmp;
    cv::Mat signatures_tmp;
    featureEncoding(*est, filtered_kp_indices, filtered_kp_indices_tmp, signatures_tmp);

    if (est->detectsKeypoints())  // for SIFT we do not need to filter keypoints after detection (which includes kp
                                  // extraction)
    {
      std::vector<int> inlier = getInlier(filtered_kp_indices_tmp);
      filtered_kp_indices_tmp = filterVector<KeypointIndex>(filtered_kp_indices_tmp, inlier);
      signatures_tmp = filterCvMat(signatures_tmp, inlier);
    }

    if (filtered_kp_indices_tmp.empty())
      continue;

    if (visualize_keypoints_)
      visualizeKeypoints(filtered_kp_indices_tmp, keypoint_indices);

    {
      pcl::StopWatch t;
      featureMatching(filtered_kp_indices_tmp, signatures_tmp, lomdbs_[est_id]);
      VLOG(1) << "Matching " << filtered_kp_indices_tmp.size() << " " << est->getFeatureDescriptorName()
              << " features (with id " << est->getUniqueId() << ") took " << t.getTime() << " ms.";
    }
    indices_.clear();
  }
  VLOG(1) << "Estimating the current feature (" << feature_descriptor_name << ") took " << t_total.getTime() << " ms.";
  indices_.clear();
}

void LocalRecognizerParameter::init(boost::program_options::options_description &desc,
                                    const std::string &section_name) {
  desc.add_options()((section_name + ".kdtree_splits").c_str(),
                     po::value<int>(&kdtree_splits_)->default_value(kdtree_splits_), "kdtree splits");
  desc.add_options()((section_name + ".kdtree_num_trees").c_str(),
                     po::value<int>(&kdtree_num_trees_)->default_value(kdtree_num_trees_),
                     "number of trees for FLANN approximate nearest neighbor search");
  desc.add_options()((section_name + ".kdtree_search_checks").c_str(),
                     po::value<int>(&kdtree_search_checks_)->default_value(kdtree_search_checks_),
                     "The number of times the tree(s) in the index should be recursively traversed. A higher value for "
                     "this parameter would give better search precision, but also take more time.");
  desc.add_options()((section_name + ".lsh_index_table_number").c_str(),
                     po::value<int>(&lsh_index_table_number_)->default_value(lsh_index_table_number_),
                     "Number of hash tables to use, usually 10-30");
  desc.add_options()((section_name + ".lsh_index_key_index").c_str(),
                     po::value<int>(&lsh_index_key_index_)->default_value(lsh_index_key_index_),
                     "key bits, usually 10-20");
  desc.add_options()((section_name + ".lsh_index_multi_probe_level").c_str(),
                     po::value<int>(&lsh_index_multi_probe_level_)->default_value(lsh_index_multi_probe_level_),
                     "controls how neighboring buckets are searched. Recommended value is 2. If set to 0, the "
                     "algorithm will degenerate into non-multiprobe LSH");
  desc.add_options()((section_name + ".knn").c_str(), po::value<size_t>(&knn_)->default_value(knn_),
                     "nearest neighbors to search for when checking feature descriptions of the scene");
  desc.add_options()((section_name + ".max_descriptor_distance").c_str(),
                     po::value<float>(&max_descriptor_distance_)->default_value(max_descriptor_distance_),
                     "maximum distance of the descriptor in the respective norm (L1 or L2) to create a correspondence");
  desc.add_options()((section_name + ".correspondence_distance_weight").c_str(),
                     po::value<float>(&correspondence_distance_weight_)->default_value(correspondence_distance_weight_),
                     "weight factor for correspondences distances. This is done to favour correspondences "
                     "from different pipelines that are more reliable than other "
                     "(SIFT and SHOT corr. simultaneously fed into CG)");
  desc.add_options()(
      (section_name + ".use_brute_force_matching").c_str(),
      po::value<bool>(&use_brute_force_matching_)->default_value(use_brute_force_matching_),
      "if true, runs brute force feature matching. Otherwise, uses FLANN approximate nearest neighbor search");
  desc.add_options()((section_name + ".distance_metric").c_str(),
                     po::value<DistanceMetric>(&distance_metric_)->default_value(distance_metric_),
                     "defines the norm used for feature matching (L1 norm, L2 norm, ChiSquare, Hellinger, Hamming)");
  desc.add_options()((section_name + ".max_keypoint_distance_z").c_str(),
                     po::value<float>(&max_keypoint_distance_z_)->default_value(max_keypoint_distance_z_),
                     "maximum distance of an extracted keypoint to be accepted");
  desc.add_options()((section_name + ".filter_planar").c_str(),
                     po::value<bool>(&filter_planar_)->default_value(filter_planar_),
                     "Filter keypoints on a planar surface");
  desc.add_options()((section_name + ".min_plane_size").c_str(),
                     po::value<int>(&min_plane_size_)->default_value(min_plane_size_),
                     "Minimum number of points for a plane to be checked if filter only points above table plane");
  desc.add_options()((section_name + ".planar_support_radius").c_str(),
                     po::value<float>(&planar_support_radius_)->default_value(planar_support_radius_),
                     "Radius used to check keypoints for planarity.");
  desc.add_options()(
      (section_name + ".threshold_planar").c_str(),
      po::value<float>(&threshold_planar_)->default_value(threshold_planar_),
      "threshold ratio used for deciding if patch is planar. Ratio defined as largest eigenvalue to all others.");
  desc.add_options()(
      (section_name + ".filter_border_pts").c_str(),
      po::value<int>(&filter_border_pts_)->default_value(filter_border_pts_),
      "Filter keypoints at the boundary (value according to the edge types defined in "
      "\"pcl::OrganizedEdgeBase (EDGELABEL_OCCLUDING  | EDGELABEL_OCCLUDED | EDGELABEL_NAN_BOUNDARY))\"");
  desc.add_options()((section_name + ".boundary_width").c_str(),
                     po::value<int>(&boundary_width_)->default_value(boundary_width_),
                     "Width in pixel of the depth discontinuity");
  desc.add_options()((section_name + ".required_viewpoint_change_deg").c_str(),
                     po::value<float>(&required_viewpoint_change_deg_)->default_value(required_viewpoint_change_deg_),
                     "required viewpoint change in degree for a new training view to be used for "
                     "feature extraction. Training views will be sorted incrementally by their "
                     "filename and if the camera pose of a training view is close to the camera "
                     "pose of an already existing training view, it will be discarded for training.");
  desc.add_options()(
      (section_name + ".train_on_individual_views").c_str(),
      po::value<bool>(&train_on_individual_views_)->default_value(train_on_individual_views_),
      "if true, extracts features from each view of the object model. Otherwise will use the full 3d cloud");
}

// template class V4R_EXPORTS LocalFeatureMatcher<pcl::PointXYZ>;
template class V4R_EXPORTS LocalFeatureMatcher<pcl::PointXYZRGB>;
}  // namespace v4r
