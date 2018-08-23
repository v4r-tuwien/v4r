#include <glog/logging.h>
#include <v4r/common/histogram.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/noise_models.h>
#include <v4r/common/normals.h>
#include <v4r/common/occlusion_reasoning.h>
#include <v4r/common/pcl_utils.h>
#include <v4r/common/time.h>
#include <v4r/common/zbuffering.h>
#include <v4r/recognition/hypotheses_verification.h>
#include <v4r/segmentation/segmenter_conditional_euclidean.h>

#include <pcl/common/time.h>
#include <pcl/registration/icp.h>
#include <pcl_1_8/keypoints/uniform_sampling.h>

#include <omp.h>
#include <opencv2/opencv.hpp>

namespace v4r {

template <typename PointT>
bool HypothesisVerification<PointT>::customRegionGrowing(const PointTWithNormal &seed_pt,
                                                         const PointTWithNormal &candidate_pt,
                                                         float squared_distance) const {
  (void)squared_distance;
  // float curvature_threshold = param_.curvature_threshold_;
  float eps_angle_threshold_rad = eps_angle_threshold_rad_;

  if (!std::isfinite(seed_pt.getNormalVector4fMap()(0)) || std::isnan(seed_pt.getNormalVector4fMap()(0)) ||
      !std::isfinite(candidate_pt.getNormalVector4fMap()(0)) || std::isnan(candidate_pt.getNormalVector4fMap()(0)))
    return false;

  if (param_.z_adaptive_) {
    float mult = std::max(seed_pt.z, 1.f);
    //            mult *= mult;
    // curvature_threshold = param_.curvature_threshold_ * mult;
    eps_angle_threshold_rad = eps_angle_threshold_rad_ * mult;
  }

  if (seed_pt.curvature > param_.curvature_threshold_)
    return false;

  if (candidate_pt.curvature > param_.curvature_threshold_)
    return false;

  float intensity_a = .2126f * seed_pt.r + .7152f * seed_pt.g + .0722f * seed_pt.b;
  float intensity_b = .2126f * candidate_pt.r + .7152f * candidate_pt.g + .0722f * candidate_pt.b;

  if (fabs(intensity_a - intensity_b) > 5.f)
    return false;

  float dotp = seed_pt.getNormalVector4fMap().dot(candidate_pt.getNormalVector4fMap());
  if (dotp < cos(eps_angle_threshold_rad))
    return false;

  return true;
}

template <typename PointT>
float HypothesisVerification<PointT>::customColorDistance(const Eigen::VectorXf &color_a,
                                                          const Eigen::VectorXf &color_b) {
  float L_dist = (color_a(0) - color_b(0)) * (color_a(0) - color_b(0));
  CHECK(L_dist >= 0.f && L_dist <= 1.f);
  L_dist /= param_.sigma_color_;
  float AB_dist = (color_a.tail(2) - color_b.tail(2)).norm();  // ( param_.color_sigma_ab_ * param_.color_sigma_ab_ );
  CHECK(AB_dist >= 0.f && AB_dist <= 1.f);
  return L_dist + AB_dist;
}

template <typename PointT>
void HypothesisVerification<PointT>::computeModelOcclusionByScene(HVRecognitionModel<PointT> &rm) const {
  bool found_model_foo;
  typename Model<PointT>::ConstPtr m = m_db_->getModelById("", rm.oh_->model_id_, found_model_foo);
  typename pcl::PointCloud<PointT>::ConstPtr model_cloud = m->getAssembled(-1);  // use full resolution for rendering
  pcl::PointCloud<pcl::Normal>::ConstPtr model_normals = m->getNormalsAssembled(-1);
  CHECK(model_cloud->points.size() == model_normals->points.size());

  const Eigen::Matrix4f hyp_tf_2_global = rm.oh_->pose_refinement_ * rm.oh_->transform_;
  typename pcl::PointCloud<PointT>::Ptr model_cloud_aligned(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*model_cloud, *model_cloud_aligned, hyp_tf_2_global);
  pcl::PointCloud<pcl::Normal>::Ptr model_normals_aligned(new pcl::PointCloud<pcl::Normal>);
  v4r::transformNormals(*model_normals, *model_normals_aligned, hyp_tf_2_global);

  boost::dynamic_bitset<> image_mask_mv(model_cloud->points.size(), 0);
  rm.image_mask_.resize(occlusion_clouds_.size(), boost::dynamic_bitset<>(occlusion_clouds_[0]->points.size(), 0));

  for (size_t view = 0; view < occlusion_clouds_.size(); view++) {
    // project into respective view
    typename pcl::PointCloud<PointT>::Ptr aligned_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr aligned_normals(new pcl::PointCloud<pcl::Normal>);
    const Eigen::Matrix4f tf = absolute_camera_poses_[view].inverse();
    pcl::transformPointCloud(*model_cloud_aligned, *aligned_cloud, tf);
    v4r::transformNormals(*model_normals_aligned, *aligned_normals, tf);

    ZBufferingParameter zBparam;
    zBparam.do_noise_filtering_ = false;
    zBparam.do_smoothing_ = false;
    zBparam.use_normals_ = true;
    ZBuffering<PointT> zbuf(cam_, zBparam);
    zbuf.setCloudNormals(aligned_normals);
    typename pcl::PointCloud<PointT>::Ptr organized_cloud_to_be_filtered(new pcl::PointCloud<PointT>);
    zbuf.renderPointCloud(*aligned_cloud, *organized_cloud_to_be_filtered, 1);
    //        std::vector<int> kept_indices = zbuf.getKeptIndices();
    Eigen::MatrixXi index_map = zbuf.getIndexMap();

    //        static pcl::visualization::PCLVisualizer vis ("self-occlusion");
    //        static int vp1, vp2, vp3, vp4;
    //        vis.removeAllPointClouds();
    //        vis.createViewPort(0,0,0.25,1,vp1);
    //        vis.createViewPort(0.25,0,0.5,1,vp2);
    //        vis.createViewPort(0.5,0,0.75,1,vp3);
    //        vis.createViewPort(0.75,0,1,1,vp4);
    //        vis.addPointCloud(aligned_cloud, "input", vp1);
    //        vis.addPointCloud(aligned_cloud, "input3", vp3);
    //        vis.addCoordinateSystem(0.04,"co",vp1);
    //        vis.addCoordinateSystem(0.04,"co2",vp2);
    //        vis.addCoordinateSystem(0.04,"co2",vp3);
    //        vis.addPointCloud(organized_cloud_to_be_filtered, "organized", vp2);
    //        boost::dynamic_bitset<> image_mask_sv( model_cloud->points.size(), 0 );
    //        for (size_t u=0; u<organized_cloud_to_be_filtered->width; u++)
    //        {
    //            for (size_t v=0; v<organized_cloud_to_be_filtered->height; v++)
    //            {
    //                int original_idx = index_map(v,u);

    //                if(original_idx < 0)
    //                    continue;
    //                image_mask_sv.set( original_idx );
    //            }
    //        }

    //        typename pcl::PointCloud<PointT>::Ptr scene_cloud_vis (new
    //        pcl::PointCloud<PointT>(*scene_cloud_downsampled_));
    //        scene_cloud_vis->sensor_orientation_ = Eigen::Quaternionf::Identity();
    //        scene_cloud_vis->sensor_origin_ = Eigen::Vector4f::Zero(4);
    //        pcl::visualization::PointCloudColorHandlerCustom<PointT> gray (scene_cloud_vis, 128, 128, 128);
    //        vis.addPointCloud(scene_cloud_vis, gray, "input_rm_vp_model_", vp1);

    //        typename pcl::PointCloud<PointT>::Ptr occlusion_cloud_view_vis (new
    //        pcl::PointCloud<PointT>(*occlusion_clouds_[view]));
    //        occlusion_cloud_view_vis->sensor_orientation_ = Eigen::Quaternionf::Identity();
    //        occlusion_cloud_view_vis->sensor_origin_ = Eigen::Vector4f::Zero(4);
    //        vis.addPointCloud(occlusion_cloud_view_vis, "occlusion_cloud_view_vis", vp3);
    //        typename pcl::PointCloud<PointT>::Ptr visible_cloud2 (new pcl::PointCloud<PointT>);
    //        pcl::copyPointCloud(*aligned_cloud, image_mask_sv, *visible_cloud2);
    //        vis.addPointCloud(visible_cloud2, "vis_cloud2", vp4);

    //        vis.spin();

    OcclusionReasoner<PointT, PointT> occ_reasoner;
    occ_reasoner.setCameraIntrinsics(cam_);
    occ_reasoner.setInputCloud(organized_cloud_to_be_filtered);
    occ_reasoner.setOcclusionCloud(occlusion_clouds_[view]);
    occ_reasoner.setOcclusionThreshold(param_.occlusion_thres_);
    boost::dynamic_bitset<> pt_is_visible = occ_reasoner.computeVisiblePoints();
    rm.image_mask_[view] = occ_reasoner.getPixelMask();

    //        {
    //            static pcl::visualization::PCLVisualizer vis2 ("occlusion_reasoning");
    //            static int vp1, vp2, vp3;
    //            vis2.removeAllPointClouds();
    //            vis2.createViewPort(0,0,0.33,1,vp1);
    //            vis2.createViewPort(0.33,0,0.66,1,vp2);
    //            vis2.createViewPort(0.66,0,1,1,vp3);
    //            vis2.addPointCloud(organized_cloud_to_be_filtered, "cloud_to_be_filtered", vp1);
    //            vis2.addPointCloud(occlusion_cloud_view_vis, "occluder_cloud", vp2);
    //            typename pcl::PointCloud<PointT>::Ptr visible_cloud (new pcl::PointCloud<PointT>);
    //            pcl::copyPointCloud(*organized_cloud_to_be_filtered, pt_is_visible, *visible_cloud);
    //            vis2.addPointCloud(visible_cloud, "vis_cloud", vp3);
    //            vis2.spin();
    //        }

    for (size_t u = 0; u < organized_cloud_to_be_filtered->width; u++) {
      for (size_t v = 0; v < organized_cloud_to_be_filtered->height; v++) {
        int idx = v * organized_cloud_to_be_filtered->width + u;

        if (!img_boundary_distance_.empty() &&
            img_boundary_distance_.at<float>(v, u) < param_.min_px_distance_to_image_boundary_)
          continue;

        if (pt_is_visible[idx]) {
          int original_idx = index_map(v, u);

          if (original_idx < 0)
            continue;

          Eigen::Vector3f viewray = aligned_cloud->points[original_idx].getVector3fMap();
          viewray.normalize();
          Eigen::Vector3f normal = aligned_normals->points[original_idx].getNormalVector3fMap();
          normal.normalize();

          float dotp = viewray.dot(normal);

          if (fabs(dotp) < param_.min_dotproduct_model_normal_to_viewray_)
            continue;

          image_mask_mv.set(original_idx);
        }
      }
    }

    //        cv::Mat registration_depth_mask = cam_->getCameraDepthRegistrationMask();

    //        cv::imshow("reg_mask", registration_depth_mask);
    //        cv::waitKey();
  }

  std::vector<int> visible_indices_tmp_full = createIndicesFromMask<int>(image_mask_mv);
  typename pcl::PointCloud<PointT>::Ptr visible_cloud_full(new pcl::PointCloud<PointT>);
  pcl::copyPointCloud(*model_cloud, visible_indices_tmp_full, *visible_cloud_full);

  // downsample
  pcl_1_8::UniformSampling<PointT> us;
  us.setRadiusSearch(param_.resolution_mm_ / 1000.0);
  us.setInputCloud(visible_cloud_full);
  pcl::PointCloud<int> sampled_indices;
  us.compute(sampled_indices);

  rm.visible_indices_.resize(sampled_indices.size());
  for (size_t i = 0; i < sampled_indices.size(); i++) {
    int idx = visible_indices_tmp_full[sampled_indices[i]];
    rm.visible_indices_[i] = idx;
  }

  rm.visible_cloud_.reset(new pcl::PointCloud<PointT>);
  rm.visible_cloud_normals_.reset(new pcl::PointCloud<pcl::Normal>);
  pcl::copyPointCloud(*model_cloud_aligned, rm.visible_indices_, *rm.visible_cloud_);
  pcl::copyPointCloud(*model_normals_aligned, rm.visible_indices_, *rm.visible_cloud_normals_);

  //    static pcl::visualization::PCLVisualizer vis;
  //    vis.removeAllPointClouds();
  //    vis.addPointCloud(rm.visible_cloud_, "scene");
  ////    vis.addPointCloud(aligned_cloud, "model");
  //    vis.spin();
}

template <typename PointT>
void HypothesisVerification<PointT>::computeVisibleOctreeNodes(HVRecognitionModel<PointT> &rm) {
  boost::dynamic_bitset<> visible_mask = v4r::createMaskFromIndices(rm.visible_indices_, rm.num_pts_full_model_);
  auto octree_it = octree_model_representation_.find(rm.oh_->model_id_);

  if (octree_it == octree_model_representation_.end())
    std::cerr << "Did not find octree representation! This should not happen!" << std::endl;

  boost::dynamic_bitset<> visible_leaf_mask(rm.num_pts_full_model_, 0);

  size_t total_leafs = 0;
  size_t visible_leafs = 0;
  for (auto leaf_it = octree_it->second->leaf_begin(); leaf_it != octree_it->second->leaf_end(); ++leaf_it) {
    pcl::octree::OctreeContainerPointIndices &container = leaf_it.getLeafContainer();

    // add points from leaf node to indexVector
    std::vector<int> indexVector;
    container.getPointIndices(indexVector);

    if (indexVector.empty())
      continue;

    total_leafs++;

    bool is_visible = false;

    for (size_t k = 0; k < indexVector.size(); k++) {
      if (visible_mask[indexVector[k]]) {
        is_visible = true;
        visible_leafs++;
        break;
      }
    }
    if (is_visible) {
      for (size_t k = 0; k < indexVector.size(); k++)
        visible_leaf_mask.set(indexVector[k]);
    }
  }
  rm.visible_indices_by_octree_ = v4r::createIndicesFromMask<int>(visible_leaf_mask);
}

template <typename PointT>
void HypothesisVerification<PointT>::refinePose(HVRecognitionModel<PointT> &rm) const {
  //    PointT minPoint, maxPoint;
  //    pcl::getMinMax3D(*rm.complete_cloud_, minPoint, maxPoint);
  //    float margin = 0.05f;
  //    minPoint.x -= margin;
  //    minPoint.y -= margin;
  //    minPoint.z -= margin;

  //    maxPoint.x += margin;
  //    maxPoint.y += margin;
  //    maxPoint.z += margin;

  //    typename pcl::PointCloud<PointT>::Ptr scene_cloud_downsampled_cropped (new pcl::PointCloud<PointT>);
  //    typename pcl::PointCloud<PointT>::Ptr scene_in_model_co (new pcl::PointCloud<PointT>);

  //    const Eigen::Matrix4f tf = rm.transform_;
  //    Eigen::Matrix4f tf_inv = tf.inverse();
  //    pcl::transformPointCloud ( *scene_cloud_downsampled_, *scene_in_model_co, tf_inv );

  //    Eigen::Matrix3f rot_tmp  = tf.block<3,3>(0,0);
  //    Eigen::Vector3f trans_tmp = tf.block<3,1>(0,3);
  //    Eigen::Affine3f affine_trans;
  //    pcl::CropBox<PointT> cropFilter;
  //    cropFilter.setInputCloud (scene_in_model_co);
  //    cropFilter.setMin(rm.model_->maxPoint_.getVector4fMap());
  //    cropFilter.setMax(rm.model_->minPoint_.getVector4fMap());
  //    affine_trans.fromPositionOrientationScale(trans_tmp, rot_tmp, Eigen::Vector3f::Ones());
  //    cropFilter.setTransform(affine_trans);
  //    cropFilter.filter (*scene_cloud_downsampled_cropped);

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(rm.visible_cloud_);
  icp.setInputTarget(scene_cloud_downsampled_);
  icp.setTransformationEpsilon(1e-6);
  icp.setMaximumIterations(param_.icp_iterations_);
  icp.setMaxCorrespondenceDistance(param_.icp_max_correspondence_);
  icp.setSearchMethodTarget(kdtree_scene_, true);
  pcl::PointCloud<PointT> aligned_visible_model;
  icp.align(aligned_visible_model);

  Eigen::Matrix4f pose_refinement;
  if (icp.hasConverged()) {
    pose_refinement = icp.getFinalTransformation();
    rm.oh_->pose_refinement_ = pose_refinement * rm.oh_->pose_refinement_;
  } else
    LOG(WARNING) << "ICP did not converge" << std::endl;

  //    static pcl::visualization::PCLVisualizer vis_tmp;
  //    static int vp2, vp3,vp1;
  //    vis_tmp.removeAllPointClouds();
  //    vis_tmp.removeAllShapes();
  //    vis_tmp.createViewPort(0,0,0.33,1, vp1);
  //    vis_tmp.createViewPort(0.33,0,0.66,1, vp2);
  //    vis_tmp.createViewPort(0.66,0,1,1, vp3);

  //    typename pcl::PointCloud<PointT>::Ptr scene_cloud_vis (new pcl::PointCloud<PointT>(*scene_cloud_downsampled_));
  //    scene_cloud_vis->sensor_orientation_ = Eigen::Quaternionf::Identity();
  //    scene_cloud_vis->sensor_origin_ = Eigen::Vector4f::Zero(4);

  //    vis_tmp.addPointCloud(rm.visible_cloud_, "model1", vp1);
  //    pcl::visualization::PointCloudColorHandlerCustom<PointT> gray (scene_cloud_vis, 128, 128, 128);
  //    vis_tmp.addPointCloud(scene_cloud_vis, gray, "scene1", vp1);
  //    vis_tmp.addText("before ICP", 10,10, 20, 1,1,1,"before_ICP",vp1);
  ////     vis_tmp.addPointCloud(rm.complete_cloud_, "model1", vp1);

  //    vis_tmp.addPointCloud(scene_cloud_vis, gray, "scene2", vp2);
  //    typename pcl::PointCloud<PointT>::Ptr model_refined (new pcl::PointCloud<PointT>);
  //    pcl::transformPointCloud(*rm.visible_cloud_, *model_refined, rm.oh_->pose_refinement_);
  //    vis_tmp.addText("after ICP", 10,10, 20, 1,1,1,"after_ICP",vp2);
  //    vis_tmp.addPointCloud(model_refined, "model2", vp2);
  //    vis_tmp.spin();

  //    vis_tmp.addPointCloud(rm.model_->getAssembled(-1), "model2", vp2);
  //    vis_tmp.addPointCloud(scene_cloud_downsampled_cropped, "scene2", vp2);
  //    vis_tmp.addSphere(rm.model_->minPoint_, 0.03, 0, 255, 0, "min", vp2 );
  //    vis_tmp.addSphere(rm.model_->maxPoint_, 0.03, 0, 255, 0, "max", vp2 );

  //    vis_tmp.addPointCloud(rm.model_->getAssembled(-1), "model", vp3);
  //    vis_tmp.addPointCloud(scene_cloud_downsampled_cropped, "scene3", vp3);

  //    typename pcl::PointCloud<PointT>::Ptr model_refined (new pcl::PointCloud<PointT>);
  //    pcl::transformPointCloud(*rm.complete_cloud_, *model_refined, refined_tf);
  //    vis_tmp.addPointCloud(model_refined, "model2", vp3);
  //    pcl::visualization::PointCloudColorHandlerCustom<PointT> gray (scene_cloud_vis, 128, 128, 128);
  //    vis_tmp.addPointCloud(scene_cloud_vis, gray, "input_rm_vp2_model_", vp3);
  //    vis_tmp.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2,
  //    "input_rm_vp2_model_", vp3);
  //    vis_tmp.addCube( minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z, maxPoint.z, 0, 1., 0, "bb", vp3);
  //    vis_tmp.spin();
}

template <typename PointT>
void HypothesisVerification<PointT>::downsampleSceneCloud() {
  if (param_.resolution_mm_ <= 0) {
    scene_cloud_downsampled_.reset(new pcl::PointCloud<PointT>(*scene_cloud_));
    scene_normals_downsampled_.reset(new pcl::PointCloud<pcl::Normal>(*scene_normals_));
  } else {
    scene_cloud_downsampled_.reset(new pcl::PointCloud<PointT>());
    scene_normals_downsampled_.reset(new pcl::PointCloud<pcl::Normal>());

    pcl_1_8::UniformSampling<PointT> us;
    us.setRadiusSearch(param_.resolution_mm_ / 1000.0);
    us.setInputCloud(scene_cloud_);
    pcl::PointCloud<int> sampled_indices;
    us.compute(sampled_indices);
    scene_sampled_indices_.resize(sampled_indices.points.size());
    for (size_t i = 0; i < scene_sampled_indices_.size(); i++)
      scene_sampled_indices_[i] = sampled_indices.points[i];

    pcl::copyPointCloud(*scene_cloud_, scene_sampled_indices_, *scene_cloud_downsampled_);
    pcl::copyPointCloud(*scene_normals_, scene_sampled_indices_, *scene_normals_downsampled_);

    for (size_t i = 0; i < scene_normals_downsampled_->points.size(); i++)
      scene_normals_downsampled_->points[i].curvature = scene_normals_->points[scene_sampled_indices_[i]].curvature;

    VLOG(1) << "Downsampled scene cloud from " << scene_cloud_->points.size() << " to "
            << scene_normals_downsampled_->points.size() << " points using uniform sampling with a resolution of "
            << param_.resolution_mm_ / 1000.0 << "m.";
  }

  removeSceneNans();

  scene_indices_map_ = Eigen::VectorXi::Constant(scene_cloud_->points.size(), -1);
  for (size_t i = 0; i < scene_sampled_indices_.size(); i++) {
    scene_indices_map_[scene_sampled_indices_[i]] = i;
  }
}

template <typename PointT>
void HypothesisVerification<PointT>::removeSceneNans() {
  CHECK(scene_cloud_downsampled_->points.size() == scene_normals_downsampled_->points.size() &&
        scene_cloud_downsampled_->points.size() == scene_sampled_indices_.size());

  size_t kept = 0;
  for (size_t i = 0; i < scene_cloud_downsampled_->points.size(); i++) {
    if (pcl::isFinite(scene_cloud_downsampled_->points[i]) && pcl::isFinite(scene_normals_downsampled_->points[i])) {
      scene_cloud_downsampled_->points[kept] = scene_cloud_downsampled_->points[i];
      scene_normals_downsampled_->points[kept] = scene_normals_downsampled_->points[i];
      scene_sampled_indices_[kept] = scene_sampled_indices_[i];
      kept++;
    }
  }
  scene_sampled_indices_.resize(kept);
  scene_cloud_downsampled_->points.resize(kept);
  scene_normals_downsampled_->points.resize(kept);
  scene_cloud_downsampled_->width = scene_normals_downsampled_->width = kept;
  scene_cloud_downsampled_->height = scene_normals_downsampled_->height = 1;
}

template <typename PointT>
void HypothesisVerification<PointT>::search() {
  pcl::StopWatch t;

  // set initial solution to hypotheses that do not have any intersection and not lie on same smooth cluster as other
  // hypothesis
  boost::dynamic_bitset<> initial_solution(global_hypotheses_.size(), 0);
  for (size_t i = 0; i < global_hypotheses_.size(); i++) {
    const auto rm = global_hypotheses_[i];
    if (intersection_cost_.row(i).sum() == 0 &&
        (!param_.check_smooth_clusters_ || smooth_region_overlap_.row(i).sum() == 0) &&
        (!param_.check_smooth_clusters_ || !rm->violates_smooth_cluster_check_)) {
      initial_solution.set(i);
    }
  }

  bool violates_smooth_region_check;
  double cost = evaluateSolution(initial_solution, violates_smooth_region_check);
  evaluated_solutions_.insert(initial_solution.to_ulong());
  if (!param_.check_smooth_clusters_ || !violates_smooth_region_check) {
    best_solution_.solution_ = initial_solution;
    best_solution_.cost_ = cost;
  } else {
    best_solution_.solution_ = boost::dynamic_bitset<>(global_hypotheses_.size(), 0);
    best_solution_.cost_ = std::numeric_limits<double>::max();
  }

  // now do a local search by enabling one hyphotheses at a time and also multiple hypotheses if they are on the same
  // smooth cluster
  bool everything_checked = false;
  evaluated_solutions_.insert(initial_solution.to_ulong());
  while (!everything_checked) {
    everything_checked = true;
    std::vector<size_t> solutions_to_be_tested;
    // flip one bit at a time
    for (size_t i = 0; i < best_solution_.solution_.size(); i++) {
      boost::dynamic_bitset<> current_solution = best_solution_.solution_;
      if (current_solution[i])
        continue;

      current_solution.flip(i);

      size_t solution_uint = current_solution.to_ulong();
      solutions_to_be_tested.push_back(solution_uint);

      // also test solutions with two new hypotheses which both describe the same smooth cluster. This should avoid
      // rejection of them if the objects are e.g. stacked together and only one smooth cluster for the stack is found.
      /// TODO: also implement checks for more than two hypotheses describing the same smooth cluster!
      if (param_.check_smooth_clusters_ && smooth_region_overlap_.row(i).sum() > 0) {
        for (size_t j = 0; j < best_solution_.solution_.size(); j++) {
          boost::dynamic_bitset<> ss = current_solution;
          if (smooth_region_overlap_(i, j) > 0 && !current_solution[j]) {
            ss.set(j);
            size_t s_uint = ss.to_ulong();
            solutions_to_be_tested.push_back(s_uint);
          }
        }
      }
    }

    for (size_t s_uint : solutions_to_be_tested) {
      // check if already evaluated
      if (evaluated_solutions_.find(s_uint) != evaluated_solutions_.end())
        continue;

      boost::dynamic_bitset<> s(global_hypotheses_.size(), s_uint);
      bool violates_smooth_region_check;
      double cost = evaluateSolution(s, violates_smooth_region_check);
      evaluated_solutions_.insert(s.to_ulong());
      if (cost < best_solution_.cost_ && (!param_.check_smooth_clusters_ || !violates_smooth_region_check)) {
        best_solution_.cost_ = cost;
        best_solution_.solution_ = s;
        everything_checked = false;
      }
    }
  }
  VLOG(1) << "Local search with " << num_evaluations_ << " evaluations took " << t.getTime() << " ms" << std::endl;
}

template <typename PointT>
double HypothesisVerification<PointT>::evaluateSolution(const boost::dynamic_bitset<> &solution,
                                                        bool &violates_smooth_region_check) {
  scene_pts_explained_solution_.clear();
  scene_pts_explained_solution_.resize(scene_cloud_downsampled_->points.size());

  for (size_t i = 0; i < global_hypotheses_.size(); i++) {
    if (!solution[i])
      continue;

    const typename HVRecognitionModel<PointT>::Ptr rm = global_hypotheses_[i];
    for (Eigen::SparseVector<float>::InnerIterator it(rm->scene_explained_weight_); it; ++it)
      scene_pts_explained_solution_[it.row()].push_back(PtFitness(it.value(), i));
  }

  for (auto spt_it = scene_pts_explained_solution_.begin(); spt_it != scene_pts_explained_solution_.end(); ++spt_it)
    std::sort(spt_it->begin(), spt_it->end());

  double scene_fit = 0., duplicity = 0.;
  Eigen::Array<bool, Eigen::Dynamic, 1> scene_pt_is_explained(scene_cloud_downsampled_->points.size());
  scene_pt_is_explained.setConstant(scene_cloud_downsampled_->points.size(), false);

  for (size_t s_id = 0; s_id < scene_cloud_downsampled_->points.size(); s_id++) {
    const std::vector<PtFitness> &s_pt = scene_pts_explained_solution_[s_id];
    if (!s_pt.empty()) {
      scene_fit += s_pt.back().fit_;  // uses the maximum value for scene explanation
      scene_pt_is_explained(s_id) = true;
    }

    if (s_pt.size() > 1)                        // two or more hypotheses explain the same scene point
      duplicity += s_pt[s_pt.size() - 2].fit_;  // uses the second best explanation
  }

  violates_smooth_region_check = false;
  if (param_.check_smooth_clusters_) {
    int max_label = scene_pt_smooth_label_id_.maxCoeff();
    for (int i = 1; i < max_label; i++)  // label "0" is for points not belonging to any smooth region
    {
      Eigen::Array<bool, Eigen::Dynamic, 1> s_pt_in_region = (scene_pt_smooth_label_id_.array() == i);
      Eigen::Array<bool, Eigen::Dynamic, 1> explained_pt_in_region =
          (s_pt_in_region.array() && scene_pt_is_explained.array());
      size_t num_explained_pts_in_region = explained_pt_in_region.count();
      size_t num_pts_in_smooth_regions = s_pt_in_region.count();

      if (num_explained_pts_in_region > param_.min_pts_smooth_cluster_to_be_epxlained_ &&
          (float)(num_explained_pts_in_region) / num_pts_in_smooth_regions < param_.min_ratio_cluster_explained_) {
        violates_smooth_region_check = true;
      }
    }
  }

  double cost = -(log(scene_fit) - param_.clutter_regularizer_ * duplicity);

  // VLOG(2) << "Evaluation of solution " << solution
  //       << (violates_smooth_region_check ? " violates smooth region check!" : "") << " cost: " << cost;

  num_evaluations_++;

  if (vis_cues_) {
    vis_cues_->visualize(this, solution, cost);
  }

  return cost;  // return the dual to our max problem
}

template <typename PointT>
void HypothesisVerification<PointT>::computeSmoothRegionOverlap() {
  smooth_region_overlap_ = Eigen::MatrixXi::Zero(global_hypotheses_.size(), global_hypotheses_.size());

  for (size_t i = 1; i < global_hypotheses_.size(); i++) {
    HVRecognitionModel<PointT> &rm_a = *global_hypotheses_[i];
    for (size_t j = 0; j < i; j++) {
      const HVRecognitionModel<PointT> &rm_b = *global_hypotheses_[j];

      if (rm_a.violates_smooth_cluster_check_ || rm_b.violates_smooth_cluster_check_) {
        size_t num_overlap = (rm_a.on_smooth_cluster_ & rm_b.on_smooth_cluster_).count();
        smooth_region_overlap_(i, j) = smooth_region_overlap_(j, i) = static_cast<int>(num_overlap);
      }
    }
  }
}

template <typename PointT>
void HypothesisVerification<PointT>::computePairwiseIntersection() {
  intersection_cost_ = Eigen::MatrixXf::Zero(global_hypotheses_.size(), global_hypotheses_.size());

  for (size_t i = 1; i < global_hypotheses_.size(); i++) {
    HVRecognitionModel<PointT> &rm_a = *global_hypotheses_[i];
    for (size_t j = 0; j < i; j++) {
      const HVRecognitionModel<PointT> &rm_b = *global_hypotheses_[j];

      size_t num_intersections = 0, total_rendered_points = 0;

      for (size_t view = 0; view < rm_a.image_mask_.size(); view++) {
        for (size_t px = 0; px < rm_a.image_mask_[view].size(); px++) {
          if (rm_a.image_mask_[view][px] && rm_b.image_mask_[view][px])
            num_intersections++;

          if (rm_a.image_mask_[view][px] || rm_b.image_mask_[view][px])
            total_rendered_points++;
        }
      }

      float conflict_cost = static_cast<float>(num_intersections) / total_rendered_points;
      intersection_cost_(i, j) = intersection_cost_(j, i) = conflict_cost;
    }

    if (!vis_pairwise_)
      rm_a.image_mask_.clear();
  }
}

template <typename PointT>
void HypothesisVerification<PointT>::removeModelsWithLowVisibility() {
  for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
    for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
      typename HVRecognitionModel<PointT>::Ptr &rm = obj_hypotheses_groups_[i][jj];
      if ((float)rm->visible_indices_by_octree_.size() / (float)rm->num_pts_full_model_ < param_.min_visible_ratio_) {
        rm->rejected_due_to_low_visibility_ = true;
        VLOG(1) << "Removed " << rm->oh_->model_id_ << " due to low visibility!";
      }
    }
  }
}

template <typename PointT>
void HypothesisVerification<PointT>::setHypotheses(std::vector<ObjectHypothesesGroup> &ohs) {
  obj_hypotheses_groups_.clear();
  obj_hypotheses_groups_.resize(ohs.size());
  for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
    const ObjectHypothesesGroup &ohg = ohs[i];

    obj_hypotheses_groups_[i].resize(ohg.ohs_.size());
    for (size_t jj = 0; jj < ohg.ohs_.size(); jj++) {
      ObjectHypothesis::Ptr oh = ohg.ohs_[jj];
      obj_hypotheses_groups_[i][jj].reset(new HVRecognitionModel<PointT>(oh));
      //            HVRecognitionModel<PointT> &hv_oh = *obj_hypotheses_groups_[i][jj];

      //            hv_oh.complete_cloud_.reset ( new pcl::PointCloud<PointT>);
      //            hv_oh.complete_cloud_normals_.reset (new pcl::PointCloud<pcl::Normal>);

      //            bool found_model;
      //            typename Model<PointT>::ConstPtr m = m_db_->getModelById("", oh->model_id_, found_model);
      //            typename pcl::PointCloud<PointT>::ConstPtr model_cloud = m->getAssembled ( param_.resolution_mm_
      //            ); pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud_const = m->getNormalsAssembled (
      //            param_.resolution_mm_ );
      //            pcl::transformPointCloud (*model_cloud, *hv_oh.complete_cloud_, oh->transform_);
      //            transformNormals(*normal_cloud_const, *hv_oh.complete_cloud_normals_, oh->transform_);
    }
  }
}

template <typename PointT>
bool HypothesisVerification<PointT>::isOutlier(const HVRecognitionModel<PointT> &rm) const {
  float visible_ratio = rm.visible_indices_by_octree_.size() / (float)rm.num_pts_full_model_;
  float thresh = param_.min_fitness_ + (param_.min_fitness_ - param_.min_fitness_high_) * (visible_ratio - 0.5f) /
                                           (0.5f - param_.min_visible_ratio_);
  float min_fitness_threshold = std::max<float>(param_.min_fitness_, std::min<float>(param_.min_fitness_high_, thresh));

  bool is_rejected = rm.oh_->confidence_ < min_fitness_threshold;

  VLOG(1) << rm.oh_->class_id_ << " " << rm.oh_->model_id_ << (is_rejected ? " is rejected" : "")
          << " with visible ratio of : " << visible_ratio << " and fitness " << rm.oh_->confidence_ << " (by thresh "
          << min_fitness_threshold << ")";
  return is_rejected;
}

template <typename PointT>
void HypothesisVerification<PointT>::removeRedundantPoses() {
  for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
    for (size_t j = 0; j < obj_hypotheses_groups_[i].size(); j++) {
      const HVRecognitionModel<PointT> &rm_a = *obj_hypotheses_groups_[i][j];

      if (!rm_a.isRejected()) {
        const Eigen::Matrix4f pose_a = rm_a.oh_->pose_refinement_ * rm_a.oh_->transform_;
        const Eigen::Vector4f centroid_a = pose_a.block<4, 1>(0, 3);
        const Eigen::Matrix3f rot_a = pose_a.block<3, 3>(0, 0);

        for (size_t ii = i; ii < obj_hypotheses_groups_.size(); ii++) {
          for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
            if (i == ii && jj <= j)
              continue;

            HVRecognitionModel<PointT> &rm_b = *obj_hypotheses_groups_[ii][jj];
            const Eigen::Matrix4f pose_b = rm_b.oh_->pose_refinement_ * rm_b.oh_->transform_;
            const Eigen::Vector4f centroid_b = pose_b.block<4, 1>(0, 3);
            const Eigen::Matrix3f rot_b = pose_b.block<3, 3>(0, 0);
            const Eigen::Matrix3f rot_diff = rot_b * rot_a.transpose();

            double rotx = std::abs(atan2(rot_diff(2, 1), rot_diff(2, 2)));
            double roty = std::abs(
                atan2(-rot_diff(2, 0), sqrt(rot_diff(2, 1) * rot_diff(2, 1) + rot_diff(2, 2) * rot_diff(2, 2))));
            double rotz = std::abs(atan2(rot_diff(1, 0), rot_diff(0, 0)));
            double dist = (centroid_a - centroid_b).norm();

            if ((dist < param_.min_Euclidean_dist_between_centroids_) &&
                (rotx < param_.min_angular_degree_dist_between_hypotheses_) &&
                (roty < param_.min_angular_degree_dist_between_hypotheses_) &&
                (rotz < param_.min_angular_degree_dist_between_hypotheses_)) {
              rm_b.rejected_due_to_similar_hypothesis_exists_ = true;

              VLOG(1) << rm_b.oh_->class_id_ << " " << rm_b.oh_->model_id_
                      << " is rejected because a similar hypothesis already exists.";
            }
          }
        }
      }
    }
  }
}

template <typename PointT>
void HypothesisVerification<PointT>::extractEuclideanClustersSmooth() {
  typename pcl::PointCloud<PointTWithNormal>::Ptr scene_w_normals(new pcl::PointCloud<PointTWithNormal>);

  if (scene_cloud_->isOrganized()) {
    pcl::concatenateFields(*scene_cloud_, *scene_normals_, *scene_w_normals);
  } else {
    pcl::concatenateFields(*scene_cloud_downsampled_, *scene_normals_downsampled_, *scene_w_normals);
  }

  boost::function<bool(const PointTWithNormal &, const PointTWithNormal &, float)> custom_f =
      boost::bind(&HypothesisVerification<PointT>::customRegionGrowing, this, _1, _2, _3);

  ConditionalEuclideanSegmenterParameter ces_param;
  ces_param.min_cluster_size_ = param_.min_points_;
  ces_param.max_cluster_size_ = std::numeric_limits<int>::max();
  ces_param.z_adaptive_ = param_.z_adaptive_;
  ces_param.cluster_tolerance_ = param_.cluster_tolerance_;

  ConditionalEuclideanSegmenter<PointTWithNormal> ces(ces_param);
  ces.setInputCloud(scene_w_normals);
  ces.setConditionFunction(custom_f);
  ces.segment();
  std::vector<std::vector<int>> clusters;
  ces.getSegmentIndices(clusters);

  // convert to downsample scene cloud indices
  if (scene_cloud_->isOrganized()) {
    size_t kept_clusters = 0;
    for (size_t i = 0; i < clusters.size(); i++) {
      size_t kept = 0;
      for (size_t j = 0; j < clusters[i].size(); j++) {
        int sidx_original = clusters[i][j];
        int new_index = scene_indices_map_[sidx_original];
        if (new_index >= 0) {
          clusters[i][kept] = new_index;
          kept++;
        }
      }
      clusters[i].resize(kept);

      if (clusters[i].size() >= param_.min_points_) {
        clusters[kept_clusters] = clusters[i];
        kept_clusters++;
      }
    }
    clusters.resize(kept_clusters);
  }

  scene_pt_smooth_label_id_ = Eigen::VectorXi::Zero(scene_cloud_downsampled_->points.size());
  for (size_t i = 0; i < clusters.size(); i++) {
    for (int sidx : clusters[i]) {
      scene_pt_smooth_label_id_(sidx) = i + 1;  // label "0" for points not belonging to any smooth region
    }
  }
  max_smooth_label_id_ = clusters.size();
}

// template<typename PointT, typename PointT>
// void
// HypothesisVerification<PointT>::computeLOffset(HVRecognitionModel<PointT> &rm)const
//{
//    Eigen::VectorXf color_s ( rm.scene_explained_weight_.nonZeros() );

//    size_t kept=0;
//    for (Eigen::SparseVector<float>::InnerIterator it(rm.scene_explained_weight_); it; ++it)
//    {
//        int sidx = it.index();
//        color_s(kept++) = scene_color_channels_(sidx,0);
//    }

//    Eigen::VectorXf color_new = specifyHistogram( rm.pt_color_.col( 0 ), color_s, 100, 0.f, 100.f );
//    rm.pt_color_.col( 0 ) = color_new;
//}

template <typename PointT>
void HypothesisVerification<PointT>::initialize() {
  global_hypotheses_.clear();

  size_t num_hypotheses = 0;
  for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++)
    num_hypotheses += obj_hypotheses_groups_[i].size();

  elapsed_time_.push_back(std::pair<std::string, float>("number of hypotheses", num_hypotheses));

  {
    ScopeTime t("Downsampling scene cloud");
    downsampleSceneCloud();
  }

  elapsed_time_.push_back(std::pair<std::string, float>("number of downsampled scene points (HV)",
                                                        scene_cloud_downsampled_->points.size()));

  if (img_boundary_distance_.empty()) {
    if (rgb_depth_overlap_.empty()) {
      LOG(WARNING) << "Depth registration mask not set. Using the whole image!";
    } else {
      VLOG(1) << "Computing distance transform to image boundary.";
      cv::distanceTransform(rgb_depth_overlap_, img_boundary_distance_, CV_DIST_L2, 5);
    }
  }

#pragma omp parallel sections
  {
#pragma omp section
    {
      ScopeTime t("Computing octree");
      octree_scene_downsampled_.reset(new pcl::octree::OctreePointCloudSearch<PointT>(param_.resolution_mm_ / 1000.0));
      octree_scene_downsampled_->setInputCloud(scene_cloud_downsampled_);
      octree_scene_downsampled_->addPointsFromInputCloud();
    }

#pragma omp section
    {
      ScopeTime t("Computing kd-tree");
      kdtree_scene_.reset(new pcl::search::KdTree<PointT>);
      kdtree_scene_->setInputCloud(scene_cloud_downsampled_);
    }

#pragma omp section
    {
      ScopeTime t("Computing octrees for model visibility computation");
      for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
        for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
          HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];
          bool found_model_foo;
          typename Model<PointT>::ConstPtr m = m_db_->getModelById("", rm.oh_->model_id_, found_model_foo);
          typename pcl::PointCloud<PointT>::ConstPtr model_cloud =
              m->getAssembled(-1);  // use full resolution for rendering
          rm.num_pts_full_model_ = model_cloud->points.size();

          auto model_octree_it = octree_model_representation_.find(rm.oh_->model_id_);
          if (model_octree_it == octree_model_representation_.end()) {
            std::shared_ptr<pcl::octree::OctreePointCloudPointVector<PointT>> octree(
                new pcl::octree::OctreePointCloudPointVector<PointT>(0.015f));
            octree->setInputCloud(model_cloud);
            octree->addPointsFromInputCloud();
            octree_model_representation_[rm.oh_->model_id_] = octree;
          }
        }
      }
    }
  }

  if (occlusion_clouds_.empty())  // we can treat single-view as multi-view case with just one view
  {
    if (scene_cloud_->isOrganized())
      occlusion_clouds_.push_back(scene_cloud_);
    else {
      ScopeTime t("Input point cloud of scene is not organized. Doing depth-buffering to get organized point cloud");
      ZBuffering<PointT> zbuf(cam_);
      typename pcl::PointCloud<PointT>::Ptr organized_cloud(new pcl::PointCloud<PointT>);
      zbuf.renderPointCloud(*scene_cloud_, *organized_cloud);
      occlusion_clouds_.push_back(organized_cloud);
    }

    absolute_camera_poses_.push_back(Eigen::Matrix4f::Identity());
  }

#pragma omp parallel sections
  {
#pragma omp section
    {
      {
        ScopeTime t("Computing visible model points (1st run)");
#pragma omp parallel for schedule(dynamic)
        for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
          for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
            HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];
            computeModelOcclusionByScene(
                rm);  // occlusion reasoning based on self-occlusion and occlusion from scene cloud(s)
          }
        }
      }

      if (param_.icp_iterations_) {
        {
          std::stringstream desc;
          desc << "Pose refinement with " << param_.icp_iterations_ << " ICP iterations";
          ScopeTime t(desc.str());
#pragma omp parallel for schedule(dynamic)
          for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
            for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
              HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];
              refinePose(rm);
            }
          }
        }

        {
          ScopeTime t("Computing visible model points (2nd run)");
#pragma omp parallel for schedule(dynamic)
          for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
            for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
              HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];
              computeModelOcclusionByScene(
                  rm);  // occlusion reasoning based on self-occlusion and occlusion from scene cloud(s)
            }
          }

          size_t num_visible_object_points = 0;
          for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
            for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
              HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];
              num_visible_object_points += rm.visible_cloud_->points.size();
            }
          }
          elapsed_time_.push_back(std::pair<std::string, float>("visible object points", num_visible_object_points));
        }
      }

      {
        ScopeTime t("Removing similar poses");
        removeRedundantPoses();
      }

      //            {
      //                // just mask out the visible normals as well
      //                for(size_t i=0; i<obj_hypotheses_groups_.size(); i++)
      //                {
      //                    for(size_t jj=0; jj<obj_hypotheses_groups_[i].size(); jj++)
      //                    {
      //                        HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];
      //                        rm.visible_cloud_normals_.reset(new pcl::PointCloud<pcl::Normal>);
      //                        pcl::copyPointCloud(*rm.complete_cloud_normals_, rm.visible_indices_,
      //                        *rm.visible_cloud_normals_);
      //                    }
      //                }
      //            }

      {  // used for checking pairwise intersection of objects (relate amount of overlapping pixel of their 2D
         // silhouette)
        ScopeTime t("Computing 2D silhouette of visible object model");
#pragma omp parallel for schedule(dynamic)
        for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
          for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
            HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];
            if (!rm.isRejected()) {
              rm.processSilhouette(param_.do_smoothing_, param_.smoothing_radius_, param_.do_erosion_,
                                   param_.erosion_radius_, static_cast<int>(cam_.w));
            }
          }
        }
      }

      {
        ScopeTime t("Computing visible octree nodes");
#pragma omp parallel for schedule(dynamic)
        for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
          for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
            HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];
            if (!rm.isRejected()) {
              computeVisibleOctreeNodes(rm);
            }
          }
        }
      }
      removeModelsWithLowVisibility();
    }

#pragma omp section
    {
      if (param_.check_smooth_clusters_) {
        ScopeTime t("Extracting smooth clusters");
        extractEuclideanClustersSmooth();
        for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
          for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
            auto &rm = *obj_hypotheses_groups_[i][jj];
            rm.on_smooth_cluster_ = boost::dynamic_bitset<>(max_smooth_label_id_, 0);
          }
        }
      }
    }

#pragma omp section
    if (!param_.ignore_color_even_if_exists_) {
      ScopeTime t("Converting scene color values");
      colorTransf_->convert(*scene_cloud_downsampled_, scene_color_channels_);
      //            scene_color_channels_.col(0) = (scene_color_channels_.col(0) -
      //            Eigen::VectorXf::Ones(scene_color_channels_.rows())*50.f) / 50.f;
      //            scene_color_channels_.col(1) = scene_color_channels_.col(1) / 150.f;
      //            scene_color_channels_.col(2) = scene_color_channels_.col(2) / 150.f;
    }
  }

  {
    ScopeTime t("Converting model color values");
    for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
      for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
        HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];

        if (!rm.isRejected()) {
          removeNanNormals(rm);

          if (!param_.ignore_color_even_if_exists_) {
            colorTransf_->convert(*rm.visible_cloud_, rm.pt_color_);
            //                        rm.pt_color_.col(0) = (rm.pt_color_.col(0) -
            //                        Eigen::VectorXf::Ones(rm.pt_color_.rows())*50.f) / 50.f;
            //                        rm.pt_color_.col(1) = rm.pt_color_.col(1) / 150.f;
            //                        rm.pt_color_.col(2) = rm.pt_color_.col(1) / 150.f;
          }
        }
      }
    }
  }

  {
    ScopeTime t("Computing model to scene fitness");
#pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
      for (size_t jj = 0; jj < obj_hypotheses_groups_[i].size(); jj++) {
        HVRecognitionModel<PointT> &rm = *obj_hypotheses_groups_[i][jj];

        if (!rm.isRejected())
          computeModelFitness(rm);
      }
    }
  }

  global_hypotheses_.resize(obj_hypotheses_groups_.size());

  // do non-maxima surpression on all hypotheses in a hypotheses group based on model fitness (i.e. select only the
  // one hypothesis in group with best model fit)
  size_t kept = 0;
  for (size_t i = 0; i < obj_hypotheses_groups_.size(); i++) {
    std::vector<typename HVRecognitionModel<PointT>::Ptr> ohg = obj_hypotheses_groups_[i];

    if (ohg.empty())
      continue;

    std::sort(ohg.begin(), ohg.end(), HVRecognitionModel<PointT>::modelFitCompare);
    global_hypotheses_[kept++] = ohg[0];

    for (size_t jj = 1; jj < ohg.size(); jj++) {
      ohg[jj]->rejected_due_to_better_hypothesis_in_group_ = true;
      VLOG(1) << ohg[jj]->oh_->class_id_ << " " << ohg[jj]->oh_->model_id_
              << " is rejected due to better hypotheses in global hypotheses group.";
    }
  }
  global_hypotheses_.resize(kept);
  obj_hypotheses_groups_.clear();  // free space

  if (vis_model_) {
    for (size_t i = 0; i < global_hypotheses_.size(); i++) {
      VLOG(1) << "Visualizing hypothesis " << i;
      vis_model_->visualize(this, *global_hypotheses_[i]);
    }
  }

  size_t kept_hypotheses = 0;
  for (size_t i = 0; i < global_hypotheses_.size(); i++) {
    const typename HVRecognitionModel<PointT>::Ptr rm = global_hypotheses_[i];

    rm->is_outlier_ = isOutlier(*rm);

    if (rm->is_outlier_)
      VLOG(1) << rm->oh_->class_id_ << " " << rm->oh_->model_id_ << " is rejected due to low model fitness score.";

    if (!rm->isRejected()) {
      VLOG(1) << rm->oh_->class_id_ << " " << rm->oh_->model_id_ << " with hypothesis id " << i
              << ", scene explained weights " << rm->scene_explained_weight_.sum() << ".";
      global_hypotheses_[kept_hypotheses++] = global_hypotheses_[i];
    } else {
      VLOG(1) << rm->oh_->class_id_ << " " << rm->oh_->model_id_ << " with hypothesis id " << i << " is rejected.";
    }
  }

  global_hypotheses_.resize(kept_hypotheses);

  if (param_.check_smooth_clusters_) {
    kept_hypotheses = 0;
    ScopeTime t("Computing smooth region intersection");
    computeSmoothRegionOverlap();

    for (size_t i = 0; i < global_hypotheses_.size(); i++) {
      HVRecognitionModel<PointT> &rm = *global_hypotheses_[i];

      if (!rm.isRejected() && rm.violates_smooth_cluster_check_ && smooth_region_overlap_.row(i).sum() == 0) {
        rm.rejected_due_to_smooth_cluster_violation = true;
        VLOG(1) << rm.oh_->class_id_ << " " << rm.oh_->model_id_ << " with hypothesis id " << i
                << " rejected due to smooth cluster violation.";
      } else {
        global_hypotheses_[kept_hypotheses++] = global_hypotheses_[i];
      }
    }
    global_hypotheses_.resize(kept_hypotheses);
  }

  elapsed_time_.push_back(std::pair<std::string, float>("hypotheses left for global optimization", kept_hypotheses));

  if (!kept_hypotheses)
    return;

  {
    ScopeTime t("Computing pairwise intersection");
    computePairwiseIntersection();
  }

  if (vis_pairwise_)
    vis_pairwise_->visualize(this);
}

template <typename PointT>
void HypothesisVerification<PointT>::optimize() {
  if (VLOG_IS_ON(1)) {
    VLOG(1) << global_hypotheses_.size()
            << " hypotheses are left for global verification after individual hypotheses "
               "rejection. These are the left hypotheses: ";
    for (size_t i = 0; i < global_hypotheses_.size(); i++)
      VLOG(1) << i << ": " << global_hypotheses_[i]->oh_->class_id_ << " " << global_hypotheses_[i]->oh_->model_id_;
  }

  evaluated_solutions_.clear();
  num_evaluations_ = 0;
  best_solution_.solution_ = boost::dynamic_bitset<>(global_hypotheses_.size(), 0);
  best_solution_.cost_ = std::numeric_limits<double>::max();
  search();

  for (size_t i = 0; i < global_hypotheses_.size(); i++) {
    if (best_solution_.solution_[i])
      global_hypotheses_[i]->oh_->is_verified_ = true;
  }

  std::stringstream info;
  info << "*****************************" << std::endl
       << "Solution: " << best_solution_.solution_ << std::endl
       << "Final cost: " << best_solution_.cost_ << std::endl
       << "Number of evaluations: " << num_evaluations_ << std::endl
       << "*****************************" << std::endl;
  VLOG(1) << info.str();
}

template <typename PointT>
void HypothesisVerification<PointT>::checkInput() {
  if (scene_cloud_->isOrganized()) {
    // check if camera calibration file is for the same image resolution as depth image
    if (cam_.w != scene_cloud_->width || cam_.h != scene_cloud_->height) {
      LOG(WARNING) << "Input cloud has different resolution (" << scene_cloud_->width << "x" << scene_cloud_->height
                   << ") than resolution stated in camera calibration file (" << cam_.w << "x" << cam_.h
                   << "). Will adjust camera calibration file accordingly.";
      cam_.adjustToSize(scene_cloud_->width, scene_cloud_->height);
    }

    // check if provided rgb_depth_overlap mask is the same size as RGB image
    if (!rgb_depth_overlap_.empty()) {
      CHECK(static_cast<int>(cam_.w) == rgb_depth_overlap_.cols && static_cast<int>(cam_.h) == rgb_depth_overlap_.rows);
    }
  } else {
    if (cam_.w != 640 || cam_.h != 480) {
      LOG(INFO) << "Input cloud is not organized(" << scene_cloud_->width << "x" << scene_cloud_->height
                << ") and resolution stated in camera calibration file (" << cam_.w << "x" << cam_.h
                << ") is not VGA! Please check if this is okay.";
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void HypothesisVerification<PointT>::verify() {
  elapsed_time_.clear();

  checkInput();

  {
    ScopeTime t("Verification of object hypotheses");
    initialize();
  }

  {
    ScopeTime t("Optimizing object hypotheses verification cost function");
    optimize();
  }

  cleanUp();
}

template <typename PointT>
bool HypothesisVerification<PointT>::removeNanNormals(HVRecognitionModel<PointT> &rm) const {
  if (!rm.visible_cloud_normals_) {
    LOG(WARNING) << "Normals are not given for input model. Need to recompute. Consider to compute normals in advance!";
    rm.visible_cloud_normals_.reset(new pcl::PointCloud<pcl::Normal>);
    model_normal_estimator_->setInputCloud(rm.visible_cloud_);
    rm.visible_cloud_normals_ = model_normal_estimator_->compute();
  }

  // check nans...
  size_t kept = 0;
  for (size_t idx = 0; idx < rm.visible_cloud_->points.size(); idx++) {
    if (pcl::isFinite(rm.visible_cloud_->points[idx]) && pcl::isFinite(rm.visible_cloud_normals_->points[idx])) {
      rm.visible_cloud_->points[kept] = rm.visible_cloud_->points[idx];
      rm.visible_cloud_normals_->points[kept] = rm.visible_cloud_normals_->points[idx];
      kept++;
    }
  }

  rm.visible_cloud_->points.resize(kept);
  rm.visible_cloud_normals_->points.resize(kept);
  rm.visible_cloud_->width = rm.visible_cloud_normals_->width = kept;
  rm.visible_cloud_->height = rm.visible_cloud_normals_->height = 1;

  return !rm.visible_cloud_->points.empty();
}

template <typename PointT>
void HypothesisVerification<PointT>::computeModelFitness(HVRecognitionModel<PointT> &rm) const {
  //    rm.model_scene_c_.reserve( rm.visible_cloud_->points.size () * param_.knn_inliers_ );
  //    size_t kept=0;

  //    pcl::visualization::PCLVisualizer vis;
  //    int vp1, vp2;
  //    vis.createViewPort(0,0,0.5,1,vp1);
  //    vis.createViewPort(0.5,0,1,1,vp2);
  //    vis.addPointCloud(rm.visible_cloud_, "vis_cloud", vp1);
  //    pcl::visualization::PointCloudColorHandlerCustom<PointT> gray (scene_cloud_downsampled_, 128, 128, 128);
  //    vis.addPointCloud(scene_cloud_downsampled_, gray, "scene1", vp1);
  //    vis.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "scene1");

  for (size_t midx = 0; midx < rm.visible_cloud_->points.size(); midx++) {
    std::vector<int> nn_indices;
    std::vector<float> nn_sqrd_distances;

    //        bool is_outlier = true;
    double radius = search_radius_;
    PointT query_pt;
    query_pt.getVector4fMap() = rm.visible_cloud_->points[midx].getVector4fMap();
    octree_scene_downsampled_->radiusSearch(query_pt, radius, nn_indices, nn_sqrd_distances);

    //        vis.addSphere(rm.visible_cloud_->points[midx], 0.005, 0., 1., 0., "queryPoint", vp1 );

    const auto normal_m = rm.visible_cloud_normals_->points[midx].getNormalVector4fMap();
    //        normal_m.normalize();

    for (size_t k = 0; k < nn_indices.size(); k++) {
      int sidx = nn_indices[k];
      float sqrd_3D_dist = nn_sqrd_distances[k];

      //            std::stringstream pt_id; pt_id << "searched_pt_" << k;
      //            vis.addSphere(scene_cloud_downsampled_->points[sidx], 0.005, 1., 0., 0., pt_id.str(), vp2 );
      //            vis.addPointCloud(rm.visible_cloud_, "vis_cloud2", vp2);
      //            vis.addPointCloud(scene_cloud_downsampled_, gray, "scene2", vp2);
      //            vis.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "scene2");

      //            if (sqr_3D_dist > ( 1.5f * 1.5f * param_.inliers_threshold_ * param_.inliers_threshold_ ) )
      //            ///TODO: consider camera's noise level
      //                continue;

      ModelSceneCorrespondence c(sidx, midx);
      c.dist_3D_ = sqrt(sqrd_3D_dist);

      const auto normal_s = scene_normals_downsampled_->points[sidx].getNormalVector4fMap();
      //            normal_s.normalize();

      c.normals_dotp_ = std::min(0.99999f, std::max(-0.99999f, normal_m.dot(normal_s)));

      //            CHECK (c.angle_surface_normals_rad_ <= M_PI) << "invalid normals: " << std::endl << normal_m <<
      //            std::endl << std::endl << normal_s << std::endl << std::endl << "dotp: " << dotp << std::endl <<
      //            "acos: " << c.angle_surface_normals_rad_ << std::endl;
      //            CHECK (c.angle_surface_normals_rad_ >= 0.f ) << "invalid normals: " << std::endl << normal_m <<
      //            std::endl << std::endl << normal_s << std::endl << std::endl << "dotp: " << dotp << std::endl <<
      //            "acos: " << c.angle_surface_normals_rad_ << std::endl;

      if (!param_.ignore_color_even_if_exists_) {
        const Eigen::VectorXf &color_m = rm.pt_color_.row(midx);
        const Eigen::VectorXf &color_s = scene_color_channels_.row(sidx);
        c.color_distance_ = color_dist_f_(color_s, color_m);
      }

      c.fitness_ = getFitness(c);
      rm.model_scene_c_.push_back(c);

      //            if(c.fitness_ > param_.min_fitness_)
      //                is_outlier=false;
    }
    //        vis.removeAllShapes(vp1);
  }

  //    vis.spin();
  //    rm.model_scene_c_.resize(kept);

  std::sort(rm.model_scene_c_.begin(), rm.model_scene_c_.end());

  if (param_.use_histogram_specification_) {
    boost::dynamic_bitset<> scene_pt_is_taken(scene_cloud_downsampled_->points.size(), 0);
    Eigen::VectorXf scene_color_for_model(scene_cloud_downsampled_->points.size());

    size_t kept = 0;
    for (const ModelSceneCorrespondence &c : rm.model_scene_c_) {
      size_t sidx = c.scene_id_;
      if (!scene_pt_is_taken[sidx]) {
        scene_pt_is_taken.set(sidx);
        scene_color_for_model(kept++) = scene_color_channels_(sidx, 0);
      }
    }
    scene_color_for_model.conservativeResize(kept);

    if (kept) {
      float mean_l_value_scene = scene_color_for_model.mean();
      float mean_l_value_model = rm.pt_color_.col(0).mean();

      float max_l_offset = 15.f;
      float l_compensation =
          std::max<float>(-max_l_offset, std::min<float>(max_l_offset, (mean_l_value_scene - mean_l_value_model)));
      //        Eigen::VectorXf color_new = specifyHistogram( rm.pt_color_.col( 0 ), scene_color_for_model, 50,
      //        min_l_value, max_l_value );
      rm.pt_color_.col(0).array() = rm.pt_color_.col(0).array() + l_compensation;

      for (ModelSceneCorrespondence &c : rm.model_scene_c_) {
        size_t sidx = c.scene_id_;
        size_t midx = c.model_id_;

        const Eigen::VectorXf &color_m = rm.pt_color_.row(midx);
        const Eigen::VectorXf &color_s = scene_color_channels_.row(sidx);
        c.color_distance_ = color_dist_f_(color_s, color_m);
        c.fitness_ = getFitness(c);
      }
    }
  }

  Eigen::Array<bool, Eigen::Dynamic, 1> scene_explained_pts(scene_cloud_downsampled_->points.size());
  scene_explained_pts.setConstant(scene_cloud_downsampled_->points.size(), false);

  Eigen::Array<bool, Eigen::Dynamic, 1> model_explained_pts(rm.visible_cloud_->points.size());
  model_explained_pts.setConstant(rm.visible_cloud_->points.size(), false);

  Eigen::VectorXf modelFit = Eigen::VectorXf::Zero(rm.visible_cloud_->points.size());
  rm.scene_explained_weight_ = Eigen::SparseVector<float>(scene_cloud_downsampled_->points.size());
  rm.scene_explained_weight_.reserve(rm.model_scene_c_.size());

  for (const ModelSceneCorrespondence &c : rm.model_scene_c_) {
    size_t sidx = c.scene_id_;
    size_t midx = c.model_id_;

    if (!scene_explained_pts(sidx)) {
      scene_explained_pts(sidx) = true;
      rm.scene_explained_weight_.insert(sidx) = c.fitness_;
    }

    if (!model_explained_pts(midx)) {
      model_explained_pts(midx) = true;
      modelFit(midx) = c.fitness_;
    }
  }

  if (param_.check_smooth_clusters_) {
    // save which smooth clusters align with hypotheses
    for (int i = 1; i < max_smooth_label_id_; i++) {  // ignore label 0 as these are unlabeled clusters (which are e.g.
      // smaller than the minimal required cluster size)
      Eigen::Array<bool, Eigen::Dynamic, 1> s_pt_in_region = (scene_pt_smooth_label_id_.array() == i);
      Eigen::Array<bool, Eigen::Dynamic, 1> explained_pt_in_region =
          (s_pt_in_region.array() && scene_explained_pts.array());
      size_t num_explained_pts_in_region = explained_pt_in_region.count();
      size_t num_pts_in_smooth_regions = s_pt_in_region.count();

      rm.on_smooth_cluster_[i] = num_explained_pts_in_region > 0;

      if (num_explained_pts_in_region > param_.min_pts_smooth_cluster_to_be_epxlained_ &&
          (float)(num_explained_pts_in_region) / num_pts_in_smooth_regions < param_.min_ratio_cluster_explained_) {
        rm.violates_smooth_cluster_check_ = true;
      }
    }
  }

  rm.model_fit_ = modelFit.sum();
  rm.oh_->confidence_ = rm.model_fit_ / rm.visible_cloud_->points.size();

  VLOG(1) << "model fit of " << rm.oh_->model_id_ << ": " << rm.model_fit_
          << " (normalized: " << rm.model_fit_ / rm.visible_cloud_->points.size() << ").";
}

template <typename PointT>
std::vector<std::pair<std::string, float>> HypothesisVerification<PointT>::elapsed_time_;

// template class V4R_EXPORTS HypothesisVerification<pcl::PointXYZ>;
template class V4R_EXPORTS HypothesisVerification<pcl::PointXYZRGB>;
}  // namespace v4r
