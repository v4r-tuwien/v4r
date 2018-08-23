#include <pcl/common/transforms.h>
#include <time.h>
#include <v4r/apps/visualization.h>
#include <v4r/recognition/model.h>

namespace v4r {

std::istream &operator>>(std::istream &in, ObjRecoVisLayoutStyle &style) {
  std::string token;
  in >> token;
  boost::to_upper(token);
  if (token == "FULL")
    style = ObjRecoVisLayoutStyle::FULL;
  else if (token == "INTERMEDIATE")
    style = ObjRecoVisLayoutStyle::INTERMEDIATE;
  else if (token == "SIMPLE")
    style = ObjRecoVisLayoutStyle::SIMPLE;
  else
    in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream &operator<<(std::ostream &out, const ObjRecoVisLayoutStyle &style) {
  switch (style) {
    case ObjRecoVisLayoutStyle::FULL:
      out << "FULL";
      break;
    case ObjRecoVisLayoutStyle::INTERMEDIATE:
      out << "INTERMEDIATE";
      break;
    case ObjRecoVisLayoutStyle::SIMPLE:
      out << "SIMPLE";
      break;
    default:
      out.setstate(std::ios_base::failbit);
  }
  return out;
}

template <typename PointT>
void ObjectRecognitionVisualizer<PointT>::pointPickingEventOccured(
    const pcl::visualization::PointPickingEvent &event) const {
  float x, y, z;
  event.getPoint(x, y, z);
  //    std::cout << "Point ID: " << event.getPointIndex() << " Clicked Point: " << x << "/" << y << "/" << z <<
  //    std::endl;

  pcl::PointXYZ searchPoint;
  searchPoint.x = x;
  searchPoint.y = y;
  searchPoint.z = z;

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  if (kdtree_ && kdtree_->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
      //          std::cout << "NN: " << pointIdxNKNSearch[i] << ", Distance: " << sqrt(pointNKNSquaredDistance[i]) <<
      //          std::endl;
      int s_kp_idx = pointIdxNKNSearch[i];

      size_t counter = 0;
      bool found = false;
      for (size_t ohg_id = 0; ohg_id < generated_object_hypotheses_.size(); ohg_id++) {
        const ObjectHypothesesGroup &ohg = generated_object_hypotheses_[ohg_id];

        for (size_t oh_id = 0; oh_id < ohg.ohs_.size(); oh_id++) {
          const ObjectHypothesis &oh = *ohg.ohs_[oh_id];

          for (const pcl::Correspondence &c : oh.corr_) {
            //                  pcl::PointXYZRGB p = cloud_->points [c.index_match];
            //                  std::cout << "Point " << counter << ": " << p.x << "/" << p.y << "/" << p.z << "
            //                  (distance: " << (p.getVector3fMap()-searchPoint.getVector3fMap()).norm() << ")" <<
            //                  std::endl;
            if ((int)counter == s_kp_idx) {
              std::cout << "Feature Distance: " << c.distance << std::endl;
              //                      pcl::PointXYZRGB p = cloud_->points [c.index_match];
              //                      std::cout << "****Found Point: " << p.x << "/" << p.y << "/" << p.z << "
              //                      (distance: " << (p.getVector3fMap()-searchPoint.getVector3fMap()).norm() << ")" <<
              //                      std::endl;

              found = true;
              break;
            }
            counter++;
          }
          //              std::cout << std::endl;
          if (found)
            break;
        }
      }
    }
  }
}

template <typename PointT>
void ObjectRecognitionVisualizer<PointT>::updateExtendedVisualization() const {
  double object_specific_opacity = 0.;
  double corr_specific_opacity = 0.;
  double scene_overlay_opacity;
  scene_overlay_ ? scene_overlay_opacity = 0.2 : scene_overlay_opacity = 0.;

  // REMOVE EXISTING LINES
  for (const Line &l : corrs_hypothesis_specific_color_) {
    vis_->removeShape(l.id_, l.viewport_);
    vis_->removeShape(l.id_);
  }

  for (const Line &l : corrs_corr_specific_color_) {
    vis_->removeShape(l.id_, l.viewport_);
    vis_->removeShape(l.id_);
  }

  switch (visualization_status) {
    case KP_VIS_STATE::NOTHING: {
      object_specific_opacity = 0.;
      corr_specific_opacity = 0.;
      break;
    }
    case KP_VIS_STATE::OBJECT_SPECIFIC_COLOR: {
      object_specific_opacity = 1.;
      corr_specific_opacity = 0.;

      if (visualize_correspondence_lines_) {
        for (const Line &l : corrs_hypothesis_specific_color_) {
          vis_->addLine(l.p_, l.q_, l.r_, l.g_, l.b_, l.id_, vp2_);
        }
      }
      break;
    }
    case KP_VIS_STATE::CORR_SPECIFIC_COLOR: {
      object_specific_opacity = 0.;
      corr_specific_opacity = 1.;

      if (visualize_correspondence_lines_) {
        for (const Line &l : corrs_hypothesis_specific_color_) {
          vis_->addLine(l.p_, l.q_, l.r_, l.g_, l.b_, l.id_, vp2_);
        }
      }
      break;
    }
  }

  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, scene_overlay_opacity,
                                         "input_vp2");
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, scene_overlay_opacity,
                                         "input_vp2b");
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, scene_overlay_opacity,
                                         "input_vp3");

  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, object_specific_opacity,
                                         "kp_cloud_scene");
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, object_specific_opacity,
                                         "kp_cloud_model");
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, corr_specific_opacity,
                                         "kp_cloud_model2");
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, corr_specific_opacity,
                                         "kp_cloud_scene2");
}

template <typename PointT>
void ObjectRecognitionVisualizer<PointT>::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event) const {
  if (event.getKeySym() == "k" && event.keyDown()) {
    double opacity;
    vis_->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "kp_cloud_scene");

    switch (visualization_status) {
      case KP_VIS_STATE::NOTHING:
        visualization_status = KP_VIS_STATE::OBJECT_SPECIFIC_COLOR;
        break;
      case KP_VIS_STATE::OBJECT_SPECIFIC_COLOR:
        visualization_status = KP_VIS_STATE::CORR_SPECIFIC_COLOR;
        break;
      case KP_VIS_STATE::CORR_SPECIFIC_COLOR:
        visualization_status = KP_VIS_STATE::NOTHING;
        break;
    }
  } else if (event.getKeySym() == "d" && event.keyDown()) {
    scene_overlay_ = !scene_overlay_;
  } else if (event.getKeySym() == "l" && event.keyDown()) {
    visualize_correspondence_lines_ = !visualize_correspondence_lines_;
  } else if (event.getKeySym() == "a" && event.keyDown()) {
    visualize_co_axes_ = !visualize_co_axes_;

    if (!visualize_co_axes_) {
      for (const auto &c : coordinate_axis_ids_) {
        vis_->removeCoordinateSystem(c.first, c.second);
      }
    } else {
      std::cout << "Object coordinate axes will be shown again for the next run!" << std::endl;
    }
  }

  updateExtendedVisualization();
}

template <typename PointT>
void ObjectRecognitionVisualizer<PointT>::setupLayout() const {
  if (!vis_) {
    vis_.reset(new pcl::visualization::PCLVisualizer("single-view recognition results"));

    float max = 0.99f;

    // create viewports for input
    if (layout_ == ObjRecoVisLayoutStyle::FULL) {
      if (processed_cloud_) {
        vis_->createViewPort(0, 0, 0.5, 0.33, vp1a_);
        vis_->createViewPort(0.5, 0, 1, 0.33, vp1b_);
      } else
        vis_->createViewPort(0, 0, 1, 0.33, vp1a_);
    } else if (layout_ == ObjRecoVisLayoutStyle::SIMPLE) {
      vis_->createViewPort(0, 0, 0.5, 1, vp1a_);
      vis_->createViewPort(max, max, 1, 1, vp1b_);
    } else if (layout_ == ObjRecoVisLayoutStyle::INTERMEDIATE) {
      vis_->createViewPort(0, 0, 0.33, 1, vp1a_);
      vis_->createViewPort(0.33, 0, 0.66, 1, vp1b_);
    }

    // create viewports for generated and verified hypotheses
    if (layout_ == ObjRecoVisLayoutStyle::FULL) {
      vis_->createViewPort(0, 0.33, 0.5, 0.66, vp2_);
      vis_->createViewPort(0.5, 0.33, 1, 0.66, vp2b_);
      vis_->createViewPort(0, 0.66, 1, 1, vp3_);
    } else if (layout_ == ObjRecoVisLayoutStyle::SIMPLE) {
      vis_->createViewPort(max, max, 1, 1, vp2_);
      vis_->createViewPort(max, max, 1, 1, vp2b_);
      vis_->createViewPort(0.5, 0, 1, 1, vp3_);
    } else if (layout_ == ObjRecoVisLayoutStyle::INTERMEDIATE) {
      vis_->createViewPort(max, max, 1, 1, vp2_);
      vis_->createViewPort(max, max, 1, 1, vp2b_);
      vis_->createViewPort(0.66, 0, 1, 1, vp3_);
    }
  }
}

template <typename PointT>
void ObjectRecognitionVisualizer<PointT>::cleanUp() const {
  corrs_hypothesis_specific_color_.clear();
  corrs_corr_specific_color_.clear();

  vis_->removeAllPointClouds();
  vis_->removeAllPointClouds(vp1a_);
  vis_->removeAllPointClouds(vp1b_);
  vis_->removeAllPointClouds(vp2b_);
  vis_->removeAllPointClouds(vp2_);
  vis_->removeAllPointClouds(vp3_);

  vis_->removeAllShapes();
}

template <typename PointT>
void ObjectRecognitionVisualizer<PointT>::visualize() const {
  setupLayout();
  cleanUp();

  size_t gen_hyps = 0;
  for (const auto &ohg : generated_object_hypotheses_)
    gen_hyps += ohg.ohs_.size();

  int fontsize = 18;

  std::stringstream gen_hyp_ss;
  gen_hyp_ss << "generated hypotheses (" << gen_hyps << ")";
  std::stringstream refined_gen_hyp_ss;
  refined_gen_hyp_ss << "(ICP-refined) generated hyp. (" << gen_hyps << ")";
  vis_->addText("input cloud", 10, 10, fontsize, 1, 1, 1, "input_test", vp1a_);

  if (processed_cloud_)
    vis_->addText("processed cloud", 10, 10, fontsize, 1, 1, 1, "processed_test", vp1b_);

  vis_->addText(gen_hyp_ss.str(), 10, 10, fontsize, 0, 0, 0, "generated hypotheses", vp2_);
  vis_->addText("l...toggle correspondence lines", 10, 50, 12, 0, 0, 0, "toggle_lines", vp2_);
  vis_->addText("k...toggle keypoints", 10, 70, 12, 0, 0, 0, "toggle_keypoints", vp2_);
  vis_->addText("d...toggle input cloud", 10, 90, 12, 0, 0, 0, "toggle_input", vp2_);
  vis_->addText("a...show object coordinate axes", 10, 110, 12, 0, 0, 0, "toggle_co_axes", vp2_);
  vis_->addText(refined_gen_hyp_ss.str(), 10, 10, fontsize - 2, 1, 1, 1, "(refined) generated hypotheses", vp2b_);

  typename pcl::PointCloud<PointT>::Ptr vis_cloud(new pcl::PointCloud<PointT>);
  pcl::copyPointCloud(*cloud_, *vis_cloud);
  vis_cloud->sensor_origin_ = Eigen::Vector4f::Zero();
  vis_cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();

  //    if(normals_)
  //        vis_->addPointCloudNormals<PointT,pcl::Normal>( processed_cloud_, normals_, 50, 0.02f, "normals", vp1b_);

#if PCL_VERSION >= 100800
  vis_->removeAllCoordinateSystems(vp2_);
  vis_->removeAllCoordinateSystems(vp2b_);
  vis_->removeAllCoordinateSystems(vp3_);
  for (const auto &c : coordinate_axis_ids_)
    vis_->removeCoordinateSystem(c.first, c.second);
  coordinate_axis_ids_.clear();
#endif

  if (vis_param_->no_text_)
    vis_->setBackgroundColor(1, 1, 1, vp1a_);
  else
    vis_->setBackgroundColor(.0f, .0f, .0f, vp1a_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kp_scene_hypothesis_specific_color(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kp_scene_corr_specific_color(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kp_model_hypothesis_specific_color, kp_model_corr_specific_color;

  if (!model_keypoints_.empty()) {
    kp_model_hypothesis_specific_color.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    kp_model_corr_specific_color.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  }
  srand(time(NULL));

  for (size_t ohg_id = 0; ohg_id < generated_object_hypotheses_.size(); ohg_id++) {
    for (size_t i = 0; i < generated_object_hypotheses_[ohg_id].ohs_.size(); i++) {
      const ObjectHypothesis &oh = *(generated_object_hypotheses_[ohg_id].ohs_[i]);
      bool found;
      const auto m = m_db_->getModelById(oh.class_id_, oh.model_id_, found);
      const std::string model_id = oh.model_id_.substr(0, oh.model_id_.length() - 4);
      std::stringstream model_label;
      model_label << model_id << "_" << ohg_id << "_" << i;
      typename pcl::PointCloud<PointT>::Ptr model_aligned(new pcl::PointCloud<PointT>());
      typename pcl::PointCloud<PointT>::Ptr model_aligned_refined(new pcl::PointCloud<PointT>());
      typename pcl::PointCloud<PointT>::ConstPtr model_cloud = m->getAssembled(3);
      pcl::transformPointCloud(*model_cloud, *model_aligned, oh.transform_);
      vis_->addPointCloud(model_aligned, model_label.str(), vp2_);
      pcl::transformPointCloud(*model_cloud, *model_aligned_refined, oh.pose_refinement_ * oh.transform_);
      vis_->addPointCloud(model_aligned_refined, model_label.str() + "refined", vp2b_);

      if (layout_ == ObjRecoVisLayoutStyle::FULL) {
        // assign unique color for each object hypothesis
        const uint8_t r = rand() % 255;
        const uint8_t g = rand() % 255;
        const uint8_t b = rand() % 255;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr kp_scene_hypothesis_specific_color_tmp(
            new pcl::PointCloud<pcl::PointXYZRGB>);  // keypoint cloud with unique color for each object hypothesis
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr kp_scene_corr_specific_color_tmp(
            new pcl::PointCloud<pcl::PointXYZRGB>);  // keypoint cloud with unique color for each keypoint
                                                     // correspondence

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr kp_model_hypothesis_specific_color_tmp, kp_model_corr_specific_color_tmp;
        if (!model_keypoints_.empty()) {
          kp_model_hypothesis_specific_color_tmp.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
          kp_model_corr_specific_color_tmp.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        }

        for (const pcl::Correspondence &c : oh.corr_) {
          pcl::PointXYZRGB p = vis_cloud->points[c.index_match];
          p.r = r;
          p.g = g;
          p.b = b;
          kp_scene_hypothesis_specific_color_tmp->points.push_back(p);

          if (!model_keypoints_.empty()) {
            pcl::PointXYZ m_kp = model_keypoints_.at(m->id_)->keypoints_->points[c.index_query];
            pcl::PointXYZRGB m_kp_color;
            m_kp_color.getVector3fMap() = m_kp.getVector3fMap();
            m_kp_color.r = r;
            m_kp_color.g = g;
            m_kp_color.b = b;
            kp_model_hypothesis_specific_color_tmp->points.push_back(m_kp_color);
          }

          // also show unique color for each correspondence
          const uint8_t rr = rand() % 255;
          const uint8_t gg = rand() % 255;
          const uint8_t bb = rand() % 255;

          p = vis_cloud->points[c.index_match];
          p.r = rr;
          p.g = gg;
          p.b = bb;
          kp_scene_corr_specific_color_tmp->points.push_back(p);

          if (!model_keypoints_.empty()) {
            pcl::PointXYZ m_kp = model_keypoints_.at(m->id_)->keypoints_->points[c.index_query];
            pcl::PointXYZRGB m_kp_color;
            m_kp_color.getVector3fMap() = m_kp.getVector3fMap();
            m_kp_color.r = rr;
            m_kp_color.g = gg;
            m_kp_color.b = bb;
            kp_model_corr_specific_color_tmp->points.push_back(m_kp_color);
          }
        }

        if (!model_keypoints_.empty()) {
          pcl::transformPointCloud(*kp_model_hypothesis_specific_color_tmp, *kp_model_hypothesis_specific_color_tmp,
                                   oh.transform_);
          pcl::transformPointCloud(*kp_model_corr_specific_color_tmp, *kp_model_corr_specific_color_tmp, oh.transform_);
          *kp_model_hypothesis_specific_color += *kp_model_hypothesis_specific_color_tmp;
          *kp_model_corr_specific_color += *kp_model_corr_specific_color_tmp;
        }
        *kp_scene_hypothesis_specific_color += *kp_scene_hypothesis_specific_color_tmp;
        *kp_scene_corr_specific_color += *kp_scene_corr_specific_color_tmp;
      }

#if PCL_VERSION >= 100800
      if (visualize_co_axes_) {
        Eigen::Matrix4f tf_tmp = oh.transform_;
        Eigen::Matrix3f rot_tmp = tf_tmp.block<3, 3>(0, 0);
        Eigen::Vector3f trans_tmp = tf_tmp.block<3, 1>(0, 3);
        Eigen::Affine3f affine_trans;
        affine_trans.fromPositionOrientationScale(trans_tmp, rot_tmp, Eigen::Vector3f::Ones());
        std::stringstream co_id;
        co_id << ohg_id << "_" << i << "vp2";
        vis_->addCoordinateSystem(0.2f, affine_trans, co_id.str(), vp2_);
        coordinate_axis_ids_.push_back(std::pair<std::string, int>(co_id.str(), vp2_));
      }
#endif
    }
  }

  size_t num_verified_hypotheses = 0;
  for (size_t ohg_id = 0; ohg_id < generated_object_hypotheses_.size(); ohg_id++) {
    for (size_t i = 0; i < generated_object_hypotheses_[ohg_id].ohs_.size(); i++) {
      const ObjectHypothesis &oh = *(generated_object_hypotheses_[ohg_id].ohs_[i]);

      if (!oh.is_verified_)
        continue;

      num_verified_hypotheses++;

      bool found;
      typename Model<PointT>::ConstPtr m = m_db_->getModelById(oh.class_id_, oh.model_id_, found);
      const std::string model_id = oh.model_id_.substr(0, oh.model_id_.length() - 4);
      std::stringstream model_label;
      model_label << model_id << "_" << ohg_id << "_" << i << "_refined";
      typename pcl::PointCloud<PointT>::Ptr model_aligned(new pcl::PointCloud<PointT>());
      typename pcl::PointCloud<PointT>::ConstPtr model_cloud = m->getAssembled(3);
      pcl::transformPointCloud(*model_cloud, *model_aligned, oh.pose_refinement_ * oh.transform_);
      vis_->addPointCloud(model_aligned, model_label.str(), vp3_);

#if PCL_VERSION >= 100800
      if (visualize_co_axes_) {
        Eigen::Matrix4f tf_tmp = oh.transform_;
        Eigen::Matrix3f rot_tmp = tf_tmp.block<3, 3>(0, 0);
        Eigen::Vector3f trans_tmp = tf_tmp.block<3, 1>(0, 3);
        Eigen::Affine3f affine_trans;
        affine_trans.fromPositionOrientationScale(trans_tmp, rot_tmp, Eigen::Vector3f::Ones());
        std::stringstream co_id;
        co_id << ohg_id << "_" << i << "vp3";
        vis_->addCoordinateSystem(0.2f, affine_trans, co_id.str(), vp3_);
        coordinate_axis_ids_.push_back(std::pair<std::string, int>(co_id.str(), vp3_));
      }
#endif
    }
  }
  std::stringstream verified_hyp_ss;
  verified_hyp_ss << "verified hypotheses (" << num_verified_hypotheses << ")";
  vis_->addText(verified_hyp_ss.str(), 10, 10, 20, 0, 0, 0, "verified hypotheses_text", vp3_);

  if (vis_param_->no_text_)
    vis_->setBackgroundColor(1, 1, 1, vp2_);
  else
    vis_->setBackgroundColor(.5f, .5f, .5f, vp2_);

  vis_->addPointCloud(vis_cloud, "input", vp1a_);

  if (processed_cloud_)
    vis_->addPointCloud(processed_cloud_, "processed_input", vp1b_);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> gray2(vis_cloud, 255, 255, 255);
  vis_->addPointCloud(vis_cloud, gray2, "input_vp2", vp2_);
  vis_->addPointCloud(vis_cloud, gray2, "input_vp2b", vp2b_);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> gray3(vis_cloud, 128, 128, 128);
  vis_->addPointCloud(vis_cloud, gray3, "input_vp3", vp3_);
  vis_->addPointCloud(kp_scene_hypothesis_specific_color, "kp_cloud_scene", vp1a_);
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "kp_cloud_scene");
  vis_->addPointCloud(kp_scene_corr_specific_color, "kp_cloud_scene2", vp1a_);
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "kp_cloud_scene2");
  vis_->setBackgroundColor(1.f, 1.f, 1.f, vp3_);

  if (!model_keypoints_.empty()) {
    vis_->addPointCloud(kp_model_hypothesis_specific_color, "kp_cloud_model", vp2_);
    vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "kp_cloud_model");
    vis_->addPointCloud(kp_model_corr_specific_color, "kp_cloud_model2", vp2_);
    vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "kp_cloud_model2");

    for (size_t c_id = 0; c_id < kp_model_corr_specific_color->points.size(); c_id++) {
      const pcl::PointXYZRGB &m = kp_model_hypothesis_specific_color->points[c_id];
      const pcl::PointXYZRGB &s = kp_scene_hypothesis_specific_color->points[c_id];
      const pcl::PointXYZRGB &m2 = kp_model_corr_specific_color->points[c_id];
      const pcl::PointXYZRGB &s2 = kp_scene_corr_specific_color->points[c_id];
      std::stringstream line_ss;
      line_ss << "line_" << c_id;
      Line l(s, m, s.r / 255., s.g / 255., s.b / 255., line_ss.str());
      Line l2(s2, m2, s2.r / 255., s2.g / 255., s2.b / 255., line_ss.str());
      corrs_hypothesis_specific_color_.push_back(l);
      corrs_corr_specific_color_.push_back(l2);
    }
  }
  updateExtendedVisualization();

  boost::function<void(const pcl::visualization::KeyboardEvent &)> f =
      boost::bind(&ObjectRecognitionVisualizer<PointT>::keyboardEventOccurred, this, _1);
  vis_->registerKeyboardCallback(f);

  boost::function<void(const pcl::visualization::PointPickingEvent &)> f2 =
      boost::bind(&ObjectRecognitionVisualizer<PointT>::pointPickingEventOccured, this, _1);
  vis_->registerPointPickingCallback(f2);

  if (kp_scene_hypothesis_specific_color && !kp_scene_hypothesis_specific_color->points.empty()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr kp_cloud_scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*kp_scene_hypothesis_specific_color, *kp_cloud_scene_xyz);
    kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree_->setInputCloud(kp_cloud_scene_xyz);
  }

  vis_->resetCamera();
  vis_->spin();

  //    normals_.reset();
}

template class V4R_EXPORTS ObjectRecognitionVisualizer<pcl::PointXYZRGB>;
}  // namespace v4r
