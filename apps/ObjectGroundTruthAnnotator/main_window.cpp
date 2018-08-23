#ifndef Q_MOC_RUN
#include "main_window.h"
#include <vtkRenderWindow.h>
#include <QKeyEvent>
#include <QMainWindow>
#include <QStringListModel>
#include "ui_mainwindow.h"
#endif

#include <glog/logging.h>
#include <opencv/highgui.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <time.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/occlusion_reasoning.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/common/zbuffering.h>
#include <v4r/io/filesystem.h>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>

namespace bf = boost::filesystem;
namespace po = boost::program_options;

namespace std {
std::ostream &operator<<(std::ostream &, const std::vector<float> &);

std::ostream &operator<<(std::ostream &os, const std::vector<float> &vec) {
  for (auto item : vec) {
    os << item << " ";
  }
  return os;
}
}  // namespace std

void writeHypothesisToFile(std::ostream &o, const std::string &object_id, const Eigen::Matrix4f &pose,
                           float visibility = 1.f);

void MainWindow::pp_callback_scene(const pcl::visualization::PointPickingEvent &event) {
  if (event.getPointIndex() == -1)
    return;

  if (selected_hypothesis_ < 0) {
    setStatus("No hypothesis selected.");
    return;
  }

  pcl::PointXYZRGB current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);

  LOG(INFO) << "picked point: " << current_point.getVector3fMap().transpose();

  bool found = false;
  for (const auto &p : picked_scene_pts_->points) {
    if ((p.getVector4fMap() - current_point.getVector4fMap()).norm() < 0.0001f) {
      found = true;
      break;
    }
  }
  if (!found) {
    current_point.r = pt_colors_.row(picked_scene_pts_->points.size())(0);
    current_point.g = pt_colors_.row(picked_scene_pts_->points.size())(1);
    current_point.b = pt_colors_.row(picked_scene_pts_->points.size())(2);
    picked_scene_pts_->points.push_back(current_point);
    updatePose();
  } else
    setStatus("Picked point already selected!");
}

void MainWindow::pp_callback_model(const pcl::visualization::PointPickingEvent &event) {
  if (event.getPointIndex() == -1)
    return;

  if (selected_hypothesis_ < 0) {
    setStatus("No hypothesis selected.");
    return;
  }

  pcl::PointXYZRGB current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);

  LOG(INFO) << "picked point: " << current_point.getVector3fMap().transpose();

  bool found = false;
  for (const auto &p : picked_model_pts_->points) {
    if ((p.getVector4fMap() - current_point.getVector4fMap()).norm() < 0.0001f) {
      found = true;
      break;
    }
  }
  if (!found) {
    current_point.r = pt_colors_.row(picked_model_pts_->points.size())(0);
    current_point.g = pt_colors_.row(picked_model_pts_->points.size())(1);
    current_point.b = pt_colors_.row(picked_model_pts_->points.size())(2);
    picked_model_pts_->points.push_back(current_point);
    updatePose();
  } else
    setStatus("Picked point already selected!");
}

void MainWindow::clear_picked_scene_points() {
  picked_scene_pts_->points.resize(0);
  pviz_->removePointCloud("picked_scene_pts", vp_scene_2_);
  m_ui->vtk_widget->update();
}

void MainWindow::clear_picked_model_points() {
  picked_model_pts_->points.resize(0);
  pviz_models_->removePointCloud("picked_model_pts", vp_model_2_);
  m_ui->vtk_widget_models->update();
}

void MainWindow::visPickedPoints() {
  // setStatus("Picked model points: ");
  //    for (size_t i=0; i<picked_model_pts_.size(); i++)
  //    {
  //        const pcl::PointXYZ &m = picked_model_pts_[i];
  //        std::stringstream color_text;
  //        color_text << std::setprecision(3) << "x: " << m.x <<  "y: " << m.y
  //        <<  "z: " << m.z << std::endl; setStatus(color_text.str(), true,
  //        picked_pt_pair_color_(i,0), picked_pt_pair_color_(i,1),
  //        picked_pt_pair_color_(i,2));
  //    }
  pviz_models_->removePointCloud("picked_model_pts", vp_model_2_);
  pviz_models_->addPointCloud(picked_model_pts_, "picked_model_pts", vp_model_2_);
  pviz_models_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "picked_model_pts",
                                                 vp_model_2_);
  m_ui->vtk_widget_models->update();
  pviz_->removePointCloud("picked_scene_pts", vp_scene_2_);
  pviz_->addPointCloud(picked_scene_pts_, "picked_scene_pts", vp_scene_2_);
  pviz_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "picked_scene_pts");
  m_ui->vtk_widget->update();

  //  setStatus("", true);
  //  setStatus("Picked scene points: ", true);
  //  for (size_t i = 0; i < picked_scene_pts_->points.size(); i++) {
  //    const pcl::PointXYZ &s = picked_scene_pts_[i];
  //    std::stringstream color_text;
  //    color_text << std::setprecision(3) << "x: " << s.x << "y: " << s.y << "z: " << s.z << std::endl;
  //    setStatus(color_text.str(), true, picked_pt_pair_color_(i, 0), picked_pt_pair_color_(i, 1),
  //              picked_pt_pair_color_(i, 2));
  //  }
}

void MainWindow::updatePose() {
  visPickedPoints();

  if (picked_model_pts_->points.size() == picked_scene_pts_->points.size() && picked_model_pts_->points.size() >= 3) {
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> t_est;
    t_est.estimateRigidTransformation(*picked_model_pts_, *picked_scene_pts_, hypotheses_[selected_hypothesis_].pose_);
    updateSelectedHypothesis(true);
  }
}

void MainWindow::clear() {
  scene_views_.clear();
  hypotheses_.clear();
  m_ui->hypothesesList->clear();
  clear_picked_scene_points();
  clear_picked_model_points();
  map_listId_2_hypotheses_id_.clear();
  selected_hypothesis_ = -1;
  scene_merged_cloud_.reset(new pcl::PointCloud<SceneT>());
  pviz_->removeAllPointClouds();
  pviz_->removeAllShapes();
  m_ui->vtk_widget->update();
}

void MainWindow::addSelectedModelCloud(const std::string &model_name, const Eigen::Matrix4f &object_pose) {
  m_ui->hypothesesList->addItem(model_name.c_str());
  m_ui->hypothesesList->scrollToBottom();

  ObjectHypothesis h;
  h.id_ = model_name;
  h.pose_ = object_pose;
  hypotheses_.push_back(h);

  pcl::PointCloud<ModelT>::Ptr cloud(new pcl::PointCloud<ModelT>);
  selected_hypothesis_ = hypotheses_.size() - 1;
  m_ui->hypothesesList->setCurrentRow(selected_hypothesis_);
  enablePoseRefinmentButtons(true);
  updateMapListIndex2HypothesisId();
  updateSelectedHypothesis();
}

void MainWindow::keyboard_callback_model(const pcl::visualization::KeyboardEvent &event) {
  if (event.getKeyCode() && event.keyDown()) {
    unsigned char key = event.getKeyCode();
    if (key == 'b') {
      if (bg_color_model_ == bg_color::BLACK) {
        pviz_models_->setBackgroundColor(255, 255, 255, vp_model_1_);
        pviz_models_->setBackgroundColor(255, 255, 255, vp_model_2_);
        bg_color_model_ = bg_color::WHITE;
      } else {
        pviz_models_->setBackgroundColor(0, 0, 0, vp_model_1_);
        pviz_models_->setBackgroundColor(0, 0, 0, vp_model_2_);
        bg_color_model_ = bg_color::BLACK;
      }
    }
  }
}

void MainWindow::keyboard_callback_scene(const pcl::visualization::KeyboardEvent &event) {
  if (event.getKeyCode() && event.keyDown()) {
    unsigned char key = event.getKeyCode();

    if (key == 'z')
      on_z_plus_clicked();
    else if (key == 'x')
      on_x_plus_clicked();
    else if (key == 'y')
      on_y_plus_clicked();
    else if (key == 'd') {
      on_remove_selected_hyp_clicked();
    }

    else if (key == 'c') {
      static bool scene_color_handler_toggle = true;

      pviz_->removePointCloud("registered_cloud_bg", vp_scene_2_);
      if (!scene_color_handler_toggle) {
        pcl::visualization::PointCloudColorHandlerCustom<SceneT> cloud_handler(scene_merged_cloud_, 255, 255, 255);
        pviz_->addPointCloud(scene_merged_cloud_, cloud_handler, "registered_cloud_bg", vp_scene_2_);
        pviz_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "registered_cloud_bg",
                                                vp_scene_2_);
        scene_color_handler_toggle = true;
      } else {
        pcl::visualization::PointCloudColorHandlerRGBField<SceneT> cloud_handler(scene_merged_cloud_);
        pviz_->addPointCloud(scene_merged_cloud_, cloud_handler, "registered_cloud_bg", vp_scene_2_);
        pviz_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "registered_cloud_bg",
                                                vp_scene_2_);
        scene_color_handler_toggle = false;
      }
    } else if (key == 'b') {
      if (bg_color_scene_ == bg_color::BLACK) {
        pviz_->setBackgroundColor(255, 255, 255, vp_scene_1_);
        pviz_->setBackgroundColor(255, 255, 255, vp_scene_2_);
        bg_color_scene_ = bg_color::WHITE;
      } else {
        pviz_->setBackgroundColor(0, 0, 0, vp_scene_1_);
        pviz_->setBackgroundColor(0, 0, 0, vp_scene_2_);
        bg_color_scene_ = bg_color::BLACK;
      }
    }
  } else if (event.keyDown()) {
    if (event.getKeySym() == "Left")
      on_x_minus_clicked();
    else if (event.getKeySym() == "Right")
      on_x_plus_clicked();
    else if (event.getKeySym() == "Up")
      on_y_minus_clicked();
    else if (event.getKeySym() == "Down")
      on_y_plus_clicked();
  }
}

void MainWindow::fillScene() {
  pviz_->removeAllPointClouds(vp_scene_1_);
  pviz_->removeAllPointClouds(vp_scene_2_);

  scene_merged_cloud_.reset(new pcl::PointCloud<SceneT>);

  std::stringstream ss;
  ss << "Number of views in this sequence: " << scene_views_.size();
  setStatus(ss.str());

  if (scene_views_.size() == 1)  // keep it organized
  {
    scene_merged_cloud_ = scene_views_[0].cloud_;
  } else {
    for (const auto &v : scene_views_) {
      pcl::PointCloud<SceneT> aligned_view;
      pcl::transformPointCloud(*v.cloud_, aligned_view, v.pose_);
      *scene_merged_cloud_ += aligned_view;
    }

    if (param_.voxel_grid_resolution_ > 0) {
      // Downsample scene cloud
      pcl::VoxelGrid<SceneT> sor;
      sor.setInputCloud(scene_merged_cloud_);
      sor.setLeafSize(param_.voxel_grid_resolution_, param_.voxel_grid_resolution_, param_.voxel_grid_resolution_);
      sor.filter(*scene_merged_cloud_);
    }
  }

  std::vector<int> foo;
  pcl::removeNaNFromPointCloud(*scene_merged_cloud_, *scene_merged_cloud_, foo);

  if (bg_kdtree_) {
    std::vector<int> leftover_pts(scene_merged_cloud_->points.size());
    size_t kept = 0;
    for (size_t i = 0; i < scene_merged_cloud_->points.size(); i++) {
      pcl::PointXYZ searchPoint;
      searchPoint.getVector3fMap() = scene_merged_cloud_->points[i].getVector3fMap();

      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if (bg_kdtree_->radiusSearch(searchPoint, param_.subtraction_radius_, pointIdxRadiusSearch,
                                   pointRadiusSquaredDistance) == 0)
        leftover_pts[kept++] = i;
    }
    leftover_pts.resize(kept);
    pcl::copyPointCloud(*scene_merged_cloud_, leftover_pts, *scene_merged_cloud_);
  }

  pviz_->addPointCloud(scene_merged_cloud_, "registered_cloud", vp_scene_1_);
  pcl::visualization::PointCloudColorHandlerCustom<SceneT> cloud_handler(scene_merged_cloud_, 255, 255, 255);
  pviz_->addPointCloud(scene_merged_cloud_, cloud_handler, "registered_cloud_bg", vp_scene_2_);
  pviz_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "registered_cloud_bg",
                                          vp_scene_2_);
  pviz_->resetCamera();
}

void MainWindow::updateMapListIndex2HypothesisId() {
  for (size_t i = 0; i < hypotheses_.size(); i++) {
    map_listId_2_hypotheses_id_[i] = hypotheses_[i].getUniqueId();
  }
}

void MainWindow::enablePoseRefinmentButtons(bool flag) {
  m_ui->remove_selected_hyp->setEnabled(flag);
  m_ui->x_plus->setEnabled(flag);
  m_ui->x_minus->setEnabled(flag);
  m_ui->y_plus->setEnabled(flag);
  m_ui->y_minus->setEnabled(flag);
  m_ui->z_plus->setEnabled(flag);
  m_ui->z_minus->setEnabled(flag);
  m_ui->xr_plus->setEnabled(flag);
  m_ui->xr_minus->setEnabled(flag);
  m_ui->yr_plus->setEnabled(flag);
  m_ui->yr_minus->setEnabled(flag);
  m_ui->zr_plus->setEnabled(flag);
  m_ui->zr_minus->setEnabled(flag);
  m_ui->trans_step_label->setEnabled(flag);
  m_ui->trans_step_te->setEnabled(flag);
  m_ui->rot_step_label->setEnabled(flag);
  m_ui->rot_step_te->setEnabled(flag);
  m_ui->icp_iter_te->setEnabled(flag);
  m_ui->icp_iter_label->setEnabled(flag);
  m_ui->icp_max_corresp_dist_te->setEnabled(flag);
  m_ui->icp_max_corresp_dist_label->setEnabled(flag);
  m_ui->icp_button->setEnabled(flag);
}

float MainWindow::getTransStep() const {
  QString trans_step_str = m_ui->trans_step_te->text();
  bool *okay = new bool();
  float trans_step = trans_step_str.toFloat(okay) / 100.f;
  if (!*okay) {
    LOG(ERROR) << "Could not convert translation step size. Is it a number? I "
                  "set it to the default value "
               << 0.05;
    trans_step = 0.05;
  }
  return trans_step;
}

float MainWindow::getRotStep() const {
  QString rot_step_str = m_ui->rot_step_te->text();
  bool *okay = new bool();
  float rot_step = rot_step_str.toFloat(okay);
  if (!*okay) {
    LOG(ERROR) << "Could not convert rotation step size. Is it a number? I set "
                  "it to the default value "
               << 5;
    rot_step = 5;
  }
  return pcl::deg2rad(rot_step);
}

void MainWindow::updateSelectedHypothesis(bool keep_picked_pts) {
  if (!keep_picked_pts) {
    clear_picked_scene_points();
    clear_picked_model_points();
  }

  if (selected_hypothesis_ >= 0) {
    pviz_->removePointCloud("highlighted");

    pcl::PointCloud<ModelT>::ConstPtr model_cloud = loaded_models_[hypotheses_[selected_hypothesis_].id_].cloud_;
    pcl::PointCloud<ModelT>::Ptr model_aligned(new pcl::PointCloud<ModelT>);
    pcl::transformPointCloud(*model_cloud, *model_aligned, hypotheses_[selected_hypothesis_].pose_);
    pviz_->removePointCloud(map_listId_2_hypotheses_id_[selected_hypothesis_], vp_scene_2_);
    pviz_->addPointCloud(model_aligned, map_listId_2_hypotheses_id_[selected_hypothesis_], vp_scene_2_);
    LOG(INFO) << "Updating " << map_listId_2_hypotheses_id_[selected_hypothesis_];

    // pcl::visualization::PointCloudColorHandlerCustom<ModelT> handler(
    //    model_cloud_transformed, 0, 255, 0);
    // pviz_->addPointCloud<ModelT>(model_cloud_transformed, handler,
    //                             "highlighted", pviz_v2_);

    Eigen::Matrix4f tf_tmp = hypotheses_[selected_hypothesis_].pose_;
    Eigen::Matrix3f rot_tmp = tf_tmp.block<3, 3>(0, 0);
    Eigen::Vector3f trans_tmp = tf_tmp.block<3, 1>(0, 3);
    Eigen::Affine3f affine_trans;
    affine_trans.fromPositionOrientationScale(trans_tmp, rot_tmp, Eigen::Vector3f::Ones());
    /// pviz_->updatePointCloudPose(unique_str.str(), affine_trans);

    pviz_->removeCoordinateSystem("hyp_co", vp_scene_2_);
    pviz_->addCoordinateSystem(0.2f, affine_trans, "hyp_co", vp_scene_2_);
  }
  m_ui->vtk_widget->update();
}

void MainWindow::on_blend_images_clicked() {
  QString rot_step_str = m_ui->rot_step_te->text();
  bool *okay = new bool();
  float alpha = m_ui->alpha_edit->text().toFloat(okay);
  if (!*okay || alpha < 0.f || alpha > 1.f) {
    alpha = 0.5f;
    LOG(ERROR) << "Could not convert alpha value. Is it a number and between 0 and 1? I set "
                  "it to the default value "
               << alpha;
  }
  float beta = (1.f - alpha);

  for (const auto &v : scene_views_) {
    cv::Mat blended;

    v4r::PCLOpenCVConverter<SceneT> pcl_opencv_converter_scene;
    pcl_opencv_converter_scene.setCameraIntrinsics(cam_);
    pcl_opencv_converter_scene.setInputCloud(v.cloud_);
    cv::Mat scene = pcl_opencv_converter_scene.getRGBImage();

    if (!hypotheses_.empty()) {
      pcl::PointCloud<ModelT>::Ptr all_models(new pcl::PointCloud<ModelT>);
      for (const auto &h : hypotheses_) {
        pcl::PointCloud<ModelT> cloud_aligned;
        pcl::transformPointCloud(*(loaded_models_[h.id_].cloud_), cloud_aligned, v.pose_.inverse() * h.pose_);
        *all_models += cloud_aligned;
      }
      pcl_opencv_converter_scene.setInputCloud(all_models);
      pcl_opencv_converter_scene.setBackgroundColor(255, 255, 255);
      cv::Mat objects = pcl_opencv_converter_scene.getRGBImage();
      cv::addWeighted(scene, alpha, objects, beta, 0.0, blended);
    } else
      blended = scene;

    std::stringstream img_title;
    img_title << "overlay for scene " << v.filename_;
    cv::imshow(img_title.str(), blended);
  }
  cv::waitKey();
  cv::destroyAllWindows();
}

void MainWindow::setStatus(const std::string &info_text, bool append, int r, int g, int b) {
  //        info_->setText(QString::fromStdString(info_text));
  std::stringstream stream;
  stream << "<font color=\"#" << std::hex << std::setfill('0') << std::setw(2) << r << std::setfill('0') << std::setw(2)
         << g << std::setfill('0') << std::setw(2) << b << "\">" << info_text << "</font>";

  if (append)
    m_ui->infoTxt->setText(m_ui->infoTxt->text() + "<br>" + QString::fromStdString(stream.str()));
  else
    m_ui->infoTxt->setText(QString::fromStdString(stream.str()));

  LOG(INFO) << info_text;
}

void MainWindow::loadTestFiles() {
  clear();
  bf::path sequence_path = input_dir_ / test_sequences_[current_sequence_id_];
  std::vector<std::string> view_names = v4r::io::getFilesInDirectory(sequence_path, ".*.pcd", true);

  for (const auto &fn : view_names) {
    View v;
    v.filename_ = fn;
    bf::path file_path = sequence_path / v.filename_;
    v.cloud_.reset(new pcl::PointCloud<SceneT>);
    pcl::io::loadPCDFile(file_path.string(), *v.cloud_);
    v.pose_ = v4r::RotTrans2Mat4f(v.cloud_->sensor_orientation_, v.cloud_->sensor_origin_);

    // reset camera pose in the cloud itself to avoid visualization issues
    v.cloud_->sensor_origin_ = Eigen::Vector4f::Zero();
    v.cloud_->sensor_orientation_ = Eigen::Quaternionf::Identity();
    scene_views_.push_back(v);
  }

  std::stringstream ss;
  ss << "Number of views in this sequence is:" << scene_views_.size();
  setStatus(ss.str());

  fillScene();
  load_model();  // check if there is already an annotation file and if, load
                 // initial hypotheses from there
}

void MainWindow::enablePrevNextButtons() {
  m_ui->next->setEnabled(current_sequence_id_ + 1 < (int)test_sequences_.size());
  m_ui->prev->setEnabled(current_sequence_id_ >= 1);
}

void MainWindow::next() {
  clear();
  current_sequence_id_++;
  loadTestFiles();
  enablePrevNextButtons();
}

void MainWindow::prev() {
  clear();
  current_sequence_id_--;
  loadTestFiles();
  enablePrevNextButtons();
}

bool MainWindow::isHypothesisRedundant(
    const std::vector<ObjectHypothesis, Eigen::aligned_allocator<ObjectHypothesis>> &existing_hypotheses,
    const ObjectHypothesis &h) const {
  for (const auto &hb : existing_hypotheses) {
    if (h.id_ != hb.id_)
      continue;

    if ((h.pose_.block<3, 1>(0, 3) - hb.pose_.block<3, 1>(0, 3)).norm() < param_.min_dist_to_keep_hypothesis_) {
      return true;
    }
  }
  return false;
}

void MainWindow::load_model() {
  // check if some (approximate) annotation files exist and if so, load them if not redundant

  std::vector<ObjectHypothesis, Eigen::aligned_allocator<ObjectHypothesis>> hypotheses_tmp;  // to check for redundancy

  for (const auto &v : scene_views_) {
    std::string f = v.filename_;
    boost::replace_last(f, ".pcd", ".anno");
    bf::path file2check = annotation_input_dir_ / test_sequences_[current_sequence_id_] / f;
    if (v4r::io::existsFile(file2check)) {
      std::ifstream f(file2check.string());
      std::string line;
      while (std::getline(f, line)) {
        std::istringstream iss(line);
        std::string model_name, occlusion_tmp;
        iss >> model_name >> occlusion_tmp;

        Eigen::Matrix4f global_object_pose;
        for (size_t i = 0; i < 16; i++)
          iss >> global_object_pose(i / 4, i % 4);

        boost::replace_last(model_name, ":", "");
        ObjectHypothesis h;
        h.id_ = model_name;
        global_object_pose = v.pose_ * global_object_pose;
        h.pose_ = global_object_pose;

        if (!isHypothesisRedundant(hypotheses_tmp, h)) {
          hypotheses_tmp.push_back(h);
          addSelectedModelCloud(h.id_, h.pose_);
        }
      }
    }
  }
}

void MainWindow::setupGUI() {
  m_ui->setupUi(this);

  // init scene visualization
  pviz_.reset(new pcl::visualization::PCLVisualizer("test_viz", false));
  pviz_->createViewPort(0, 0, 1, 0.5, vp_scene_1_);
  pviz_->createViewPort(0, 0.5, 1, 1, vp_scene_2_);
  pviz_->addText("scenes", 10, 10, param_.fontsize_, 1., 1., 1., "scene_txt", vp_scene_1_);
  pviz_->addText("labelled objects", 10, 10, param_.fontsize_, 1., 1., 1., "labelled_objects_txt", vp_scene_2_);

  boost::function<void(const pcl::visualization::KeyboardEvent &)> fk =
      boost::bind(&MainWindow::keyboard_callback_scene, this, _1);
  pviz_->registerKeyboardCallback(fk);
  boost::function<void(const pcl::visualization::PointPickingEvent &)> fs =
      boost::bind(&MainWindow::pp_callback_scene, this, _1);
  pviz_->registerPointPickingCallback(fs);

  m_ui->vtk_widget->SetRenderWindow(pviz_->getRenderWindow());
  pviz_->setupInteractor(m_ui->vtk_widget->GetInteractor(), m_ui->vtk_widget->GetRenderWindow());
  m_ui->vtk_widget->update();

  // init model visualization
  pviz_models_.reset(new pcl::visualization::PCLVisualizer("viz_models", false));
  pviz_models_->createViewPort(0, 0, 0.5, 1, vp_model_1_);
  pviz_models_->createViewPort(0.5, 0, 1, 1, vp_model_2_);
  pviz_models_->addText("", 10, 10, param_.fontsize_, 0, 0, 0, "model_vis_text", vp_model_1_);
  boost::function<void(const pcl::visualization::PointPickingEvent &)> f =
      boost::bind(&MainWindow::pp_callback_model, this, _1);
  pviz_models_->registerPointPickingCallback(f);
  boost::function<void(const pcl::visualization::KeyboardEvent &)> fkm =
      boost::bind(&MainWindow::keyboard_callback_model, this, _1);
  pviz_models_->registerKeyboardCallback(fkm);

  // create window for models
  // m_ui->vtk_widget_models->resize(loaded_models_.size() * model_xsize_, model_xsize_);
  m_ui->vtk_widget_models->SetRenderWindow(pviz_models_->getRenderWindow());
  pviz_models_->setupInteractor(m_ui->vtk_widget_models->GetInteractor(), m_ui->vtk_widget_models->GetRenderWindow());
  m_ui->vtk_widget_models->update();

  srand(time(NULL));
  pt_colors_ = Eigen::MatrixXi::Zero(10, 3);
  pt_colors_.row(0) << 255, 0, 0;
  pt_colors_.row(1) << 0, 255, 0;
  pt_colors_.row(2) << 0, 0, 255;
  pt_colors_.row(3) << 255, 0, 255;
  pt_colors_.row(4) << 255, 255, 0;
  pt_colors_.row(5) << 0, 255, 255;
  pt_colors_.row(6) << 255, 255, 255;
  pt_colors_.row(7) << 128, 0, 255;
  pt_colors_.row(8) << 128, 128, 128;
  pt_colors_.row(9) << 0, 128, 128;

  picked_model_pts_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  picked_scene_pts_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

MainWindow::MainWindow(int argc, char *argv[], const Parameter &p)
: QMainWindow(0), m_ui(new Ui::MainWindow), param_(p) {
  FLAGS_logtostderr = 1;
  google::InitGoogleLogging(argv[0]);

  // params
  current_sequence_id_ = -1;
  output_dir_ = ".";
  bf::path bg_cloud = "";
  bf::path camera_calibration_file = v4r::io::getConfigDir() / "rgb_calibration.yaml";
  bf::path depth_registration_mask_fn;

  po::options_description desc(
      "Ground-Truth Annotation "
      "Tool\n======================================\n*"
      "*Allowed options");
  desc.add_options()("help,h", "produce help message");
  desc.add_options()("models_dir,m", po::value<bf::path>(&dir_models_)->required(),
                     "directory containing the model .pcd files");
  desc.add_options()("input_dir,i", po::value<bf::path>(&input_dir_)->required(),
                     "Input directory with the to be annotated scenes stored as point clouds "
                     "(.pcd). The camera pose is taken directly from the pcd header fields "
                     "\"sensor_orientation_\" and \"sensor_origin_\" and it also looks for an "
                     "initial annotation file \"results_3d.txt\". If this annotation file "
                     "exists, the program reads each row as \"\\path\\to\\object_model.pcd "
                     "t11 t12 t13 t14 t21 ...  t34 .. 0 0 0 1\" where t are elements of the "
                     "4x4 homogenous transformation matrix bringing the model into the world "
                     "coordinate system. If the test directory contains subdirectories, each "
                     "subdirectory is considered as separate sequence for multiview "
                     "recognition.");
  desc.add_options()("output_dir,o", po::value<bf::path>(&output_dir_)->default_value(output_dir_), "Output directory");
  desc.add_options()("annotation_input,a",
                     po::value<bf::path>(&annotation_input_dir_)->default_value(annotation_input_dir_),
                     "Directory containing some annotation hypotheses (.anno "
                     "files). These hypotheses will be loaded during start"
                     "to reduce annotation cost.");
  desc.add_options()("icp_scene_to_model", po::bool_switch(&param_.icp_scene_to_model_),
                     "if set, does pose refinement from scene to model. "
                     "Otherwise, the other way around.");
  desc.add_options()("bg_cloud", po::value<bf::path>(&bg_cloud)->default_value(bg_cloud),
                     "File path to background point cloud. This point cloud will be "
                     "subtracted from all input clouds to make ICP and point picking a bit "
                     "easier. If the file path is empty or does not exist, background "
                     "subtraction will be disabled.");
  desc.add_options()("subtraction_radius",
                     po::value<float>(&param_.subtraction_radius_)->default_value(param_.subtraction_radius_),
                     "Subtraction radius in meter used for background subtraction.");
  desc.add_options()("voxel_grid_resolution",
                     po::value<float>(&param_.voxel_grid_resolution_)->default_value(param_.voxel_grid_resolution_),
                     "resolution in meter of downsampled registered scene cloud. If negative, uses all points.");
  desc.add_options()("camera_calibration_file",
                     po::value<bf::path>(&camera_calibration_file)->default_value(camera_calibration_file),
                     "Calibration file of RGB Camera containing intrinsic parameters");
  desc.add_options()("rgbd_registration_mask",
                     po::value<bf::path>(&depth_registration_mask_fn)->default_value(depth_registration_mask_fn),
                     "File path to camera mask that tells us which pixels have valid depth values and which ones are "
                     "not seen due to the physical displacement between RGB and depth sensor. Valid pixels are set to "
                     "255, pixels that are outside depth camera's field of view are set to 0");
  desc.add_options()("visualize_occlusion_computation", po::bool_switch(&param_.visualize_occlusion_computation_),
                     "if set, visualizes occlusion computation for individual hypotheses when saving annotation.");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return;
  }
  try {
    po::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return;
  }

  try {
    cam_ = v4r::Intrinsics::load(camera_calibration_file.string());
  } catch (const std::runtime_error &e) {
    LOG(WARNING) << "Failed to load camera calibration file from " << camera_calibration_file.string()
                 << "! Will use Primesense default camera intrinsics parameters!" << std::endl;
    cam_ = v4r::Intrinsics::PrimeSense();
  }

  setupGUI();

  cv::Mat_<uchar> img_mask;
  if (v4r::io::existsFile(depth_registration_mask_fn)) {
    img_mask = cv::imread(depth_registration_mask_fn.string(), CV_LOAD_IMAGE_GRAYSCALE);
  }

  if (img_mask.empty()) {
    LOG(WARNING) << "Depth registration mask not set. Using the whole image!";
  } else {
    LOG(INFO) << "Computing distance transform to image boundary.";
    cv::distanceTransform(img_mask, img_boundary_distance_, CV_DIST_L2, 5);
  }

  // get all models that are to be annotated
  std::vector<std::string> model_fns = v4r::io::getFilesInDirectory(dir_models_, "3D_model.pcd", true);
  for (size_t i = 0; i < model_fns.size(); i++) {
    bf::path model3D_path = dir_models_ / model_fns[i];
    bf::path tmp = model_fns[i];
    const std::string model_name = tmp.parent_path().string();
    pcl::PointCloud<ModelTWithNormal>::Ptr cloud_with_normal(new pcl::PointCloud<ModelTWithNormal>);
    pcl::io::loadPCDFile(model3D_path.string(), *cloud_with_normal);
    pcl::PointCloud<ModelT>::Ptr cloud(new pcl::PointCloud<ModelT>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*cloud_with_normal, *cloud);
    pcl::copyPointCloud(*cloud_with_normal, *normals);
    LOG(INFO) << "Loaded model " << model_name;
    loaded_models_[model_name].cloud_ = cloud;
    loaded_models_[model_name].cloud_normals_ = normals;
    loaded_models_[model_name].octree_.reset(new pcl::octree::OctreePointCloudPointVector<ModelT>(0.01f));
    loaded_models_[model_name].octree_->setInputCloud(cloud);
    loaded_models_[model_name].octree_->addPointsFromInputCloud();
    m_ui->modelList->addItem(model_name.c_str());
  }

  test_sequences_ = v4r::io::getFoldersInDirectory(input_dir_);
  if (test_sequences_.empty()) {  // there is only one multi-view sequence which is
    // the current folder
    test_sequences_.push_back("");
  }

  selected_hypothesis_ = -1;

  enablePoseRefinmentButtons(false);

  if (v4r::io::existsFile(bg_cloud)) {
    pcl::PointCloud<SceneT>::Ptr background_cloud(new pcl::PointCloud<SceneT>);
    if (pcl::io::loadPCDFile(bg_cloud.string(), *background_cloud) == -1) {
      setStatus("Could not read background cloud from " + bg_cloud.string() + "!");
      background_cloud.reset();
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr background_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*background_cloud, *background_cloud_xyz),
          bg_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
      bg_kdtree_->setInputCloud(background_cloud_xyz);
    }
  }

  next();
}

void MainWindow::on_x_plus_clicked() {
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 1>(0, 3) = Eigen::Vector3f(getTransStep(), 0, 0);

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_x_minus_clicked() {
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 1>(0, 3) = Eigen::Vector3f(-getTransStep(), 0, 0);

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_y_plus_clicked() {
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 1>(0, 3) = Eigen::Vector3f(0, getTransStep(), 0);

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_y_minus_clicked() {
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 1>(0, 3) = Eigen::Vector3f(0, -getTransStep(), 0);

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_z_plus_clicked() {
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 1>(0, 3) = Eigen::Vector3f(0, 0, getTransStep());

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_z_minus_clicked() {
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 1>(0, 3) = Eigen::Vector3f(0, 0, -getTransStep());

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void writeHypothesisToFile(std::ostream &o, const std::string &object_id, const Eigen::Matrix4f &pose,
                           float visibility) {
  o << object_id << " (" << visibility << "): ";

  for (int v = 0; v < 4; v++) {
    for (int u = 0; u < 4; u++) {
      o << pose(v, u) << " ";
    }
  }
  o << std::endl;
}

float MainWindow::getObjectVisibility(const ObjectHypothesis &h, const View &v) {
  const Eigen::Matrix4f tf = v.pose_.inverse() * h.pose_;
  pcl::PointCloud<ModelT>::Ptr aligned_model(new pcl::PointCloud<ModelT>);
  pcl::PointCloud<pcl::Normal>::Ptr aligned_model_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::transformPointCloud(*loaded_models_[h.id_].cloud_, *aligned_model, tf);
  v4r::transformNormals(*loaded_models_[h.id_].cloud_normals_, *aligned_model_normals, tf);

  // compute model self-occlusion
  v4r::ZBufferingParameter zbufParam;
  zbufParam.do_smoothing_ = false;
  zbufParam.do_noise_filtering_ = false;
  zbufParam.use_normals_ = true;
  v4r::ZBuffering<ModelT> zbuf(cam_, zbufParam);
  typename pcl::PointCloud<ModelT>::Ptr organized_cloud_to_be_filtered(new pcl::PointCloud<ModelT>);
  zbuf.setCloudNormals(aligned_model_normals);
  zbuf.renderPointCloud(*aligned_model, *organized_cloud_to_be_filtered);
  std::vector<int> visible_pt_indices = zbuf.getKeptIndices();
  Eigen::MatrixXi index_map = zbuf.getIndexMap();

  // compute occlusion caused by scene
  v4r::OcclusionReasoner<SceneT, ModelT> occ_reasoner;
  occ_reasoner.setCameraIntrinsics(cam_);
  occ_reasoner.setInputCloud(organized_cloud_to_be_filtered);
  occ_reasoner.setOcclusionCloud(v.cloud_);
  occ_reasoner.setOcclusionThreshold(param_.occlusion_threshold_);
  boost::dynamic_bitset<> organized_pt_is_visible = occ_reasoner.computeVisiblePoints();

  boost::dynamic_bitset<> visible_mask(aligned_model->points.size(), 0);

  for (size_t u = 0; u < organized_cloud_to_be_filtered->width; u++) {
    for (size_t v = 0; v < organized_cloud_to_be_filtered->height; v++) {
      int idx = v * organized_cloud_to_be_filtered->width + u;

      if (!img_boundary_distance_.empty() &&
          img_boundary_distance_.at<float>(v, u) < param_.min_px_distance_to_image_boundary_)
        continue;

      if (organized_pt_is_visible[idx]) {
        int original_idx = index_map(v, u);

        if (original_idx < 0)
          continue;

        Eigen::Vector3f viewray = aligned_model->points[original_idx].getVector3fMap();
        viewray.normalize();
        Eigen::Vector3f normal = aligned_model_normals->points[original_idx].getNormalVector3fMap();
        normal.normalize();

        float dotp = viewray.dot(normal);

        if (fabs(dotp) < param_.min_dotproduct_model_normal_to_viewray_)
          continue;

        visible_mask.set(original_idx);
      }
    }
  }

  boost::dynamic_bitset<> visible_leaf_mask(aligned_model->points.size(), 0);

  size_t total_leafs = 0;
  size_t visible_leafs = 0;

  for (auto leaf_it = loaded_models_[h.id_].octree_->leaf_begin(); leaf_it != loaded_models_[h.id_].octree_->leaf_end();
       ++leaf_it) {
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
  std::vector<int> visible_indices_by_octree = v4r::createIndicesFromMask<int>(visible_leaf_mask);

  if (param_.visualize_occlusion_computation_) {
    static pcl::visualization::PCLVisualizer vis("visibility");
    vis.removeAllPointClouds();
    int vp1, vp2, vp3, vp4;
    vis.createViewPort(0, 0, 0.5, 0.5, vp1);
    vis.createViewPort(0.5, 0, 1, 0.5, vp2);
    vis.createViewPort(0, 0.5, 0.5, 1, vp3);
    vis.createViewPort(0.5, 0.5, 1, 1, vp4);
    vis.addPointCloud<SceneT>(v.cloud_, "scene", vp1);
    vis.addPointCloud<SceneT>(v.cloud_, "scene_vp3", vp3);

    pcl::PointCloud<ModelT> vis_cloud;
    pcl::copyPointCloud(*aligned_model, visible_pt_indices, vis_cloud);

    pcl::PointCloud<ModelT> vis_cloud2;
    pcl::copyPointCloud(*aligned_model, visible_indices_by_octree, vis_cloud2);

    pcl::PointCloud<ModelT> vis_cloud3;
    std::vector<int> visible_indices_after_scene_occlusion_check_ = v4r::createIndicesFromMask<int>(visible_mask);
    pcl::copyPointCloud(*aligned_model, visible_indices_after_scene_occlusion_check_, vis_cloud3);

    vis.addPointCloud(aligned_model, "full_object_cloud", vp1);
    vis.addPointCloud(vis_cloud.makeShared(), "visible_object", vp1);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0., 1., 0., "visible_object", vp1);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1., 0., 0., "full_object_cloud",
                                         vp1);

    vis.addPointCloud(aligned_model, "full_object_cloud2", vp2);
    vis.addPointCloud(vis_cloud2.makeShared(), "visible_object2", vp2);
    // vis.addPointCloud(organized_cloud, "rendered object", vp2);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0., 1., 0., "visible_object2", vp2);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1., 0., 0., "full_object_cloud2",
                                         vp2);

    vis.addPointCloud(aligned_model, "full_object_cloud3", vp3);
    vis.addPointCloud(vis_cloud3.makeShared(), "visible_object3", vp3);
    // vis.addPointCloud(organized_cloud, "rendered object", vp2);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0., 1., 0., "visible_object3", vp3);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1., 0., 0., "full_object_cloud3",
                                         vp3);

    LOG(INFO) << "Nr points full: " << aligned_model->points.size() << ", rendered: " << visible_pt_indices.size()
              << "/" << visible_indices_by_octree.size()
              << "; organized_pt_is_visible_count: " << organized_pt_is_visible.count() << " --> "
              << static_cast<float>(visible_indices_by_octree.size()) / aligned_model->points.size();
    vis.spin();
  }

  return static_cast<float>(visible_indices_by_octree.size()) / aligned_model->points.size();
}

void MainWindow::on_save_model_clicked() {
  for (const auto &v : scene_views_) {
    std::string output_file = v.filename_;
    boost::replace_last(output_file, ".pcd", ".anno");
    bf::path output_path = output_dir_ / test_sequences_[current_sequence_id_] / output_file;
    v4r::io::createDirForFileIfNotExist(output_path);

    std::ofstream of(output_path.string());
    for (const auto &h : hypotheses_) {
      float obj_visibility = getObjectVisibility(h, v);
      writeHypothesisToFile(of, h.id_, v.pose_.inverse() * h.pose_, obj_visibility);
    }
    of.close();
    setStatus("Saved annotation(s) to " + output_path.string());
  }
}

void MainWindow::on_prev_clicked() {
  prev();
}

void MainWindow::on_next_clicked() {
  next();
}

void MainWindow::on_xr_plus_clicked() {
  float rot_step = getRotStep();
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 3>(0, 0) = Eigen::Matrix3f(Eigen::AngleAxisf(rot_step, Eigen::Vector3f::UnitX()));

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_xr_minus_clicked() {
  float rot_step = getRotStep();
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 3>(0, 0) = Eigen::Matrix3f(Eigen::AngleAxisf(-rot_step, Eigen::Vector3f::UnitX()));

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_yr_plus_clicked() {
  float rot_step = getRotStep();
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 3>(0, 0) = Eigen::Matrix3f(Eigen::AngleAxisf(-rot_step, Eigen::Vector3f::UnitX()));

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_yr_minus_clicked() {
  float rot_step = getRotStep();
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 3>(0, 0) = Eigen::Matrix3f(Eigen::AngleAxisf(-rot_step, Eigen::Vector3f::UnitY()));

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_zr_plus_clicked() {
  float rot_step = getRotStep();
  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(rot_step, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 3>(0, 0) = m;

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_zr_minus_clicked() {
  float rot_step = getRotStep();
  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(-rot_step, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<3, 3>(0, 0) = m;

  if (selected_hypothesis_ >= 0) {
    hypotheses_[selected_hypothesis_].pose_ = hypotheses_[selected_hypothesis_].pose_ * m4f;
    updateSelectedHypothesis();
  }
}

void MainWindow::on_remove_selected_hyp_clicked() {
  LOG(INFO) << "Removed selected hypothesis " << selected_hypothesis_ << " from the " << hypotheses_.size()
            << " hypotheses present in total. ";

  if (selected_hypothesis_ < 0)
    return;

  pviz_->removePointCloud("highlighted");
  LOG(INFO) << "Removing " << hypotheses_[selected_hypothesis_].getUniqueId();
  pviz_->removePointCloud(hypotheses_[selected_hypothesis_].getUniqueId(), vp_scene_2_);
  pviz_->removeCoordinateSystem("hyp_co", vp_scene_2_);

  m_ui->hypothesesList->takeItem(selected_hypothesis_);
  hypotheses_.erase(hypotheses_.begin() + selected_hypothesis_);

  selected_hypothesis_ = -1;
  updateMapListIndex2HypothesisId();
  enablePoseRefinmentButtons(false);
  clear_picked_scene_points();
  clear_picked_model_points();
  m_ui->vtk_widget->update();
}

void MainWindow::on_hypothesesList_clicked(const QModelIndex &index) {
  const std::string model_name = m_ui->hypothesesList->currentItem()->text().toStdString();
  LOG(INFO) << "hypotheses list clicked..." << index.row() << " - " << model_name;
  updateModelWindow(model_name);
  enablePoseRefinmentButtons(true);
  clear_picked_scene_points();
  selected_hypothesis_ = index.row();
  updateSelectedHypothesis(false);
}

void MainWindow::updateModelWindow(const std::string &model_name) {
  pviz_models_->removeAllPointClouds();
  pviz_models_->updateText(model_name, 10, 10, param_.fontsize_, 128, 128, 128, "model_vis_text");
  pviz_models_->addPointCloud(loaded_models_[model_name].cloud_, model_name, vp_model_1_);
  pviz_models_->addPointCloud(loaded_models_[model_name].cloud_, model_name + "vp2", vp_model_2_);
  m_ui->vtk_widget_models->update();
}

void MainWindow::on_modelList_clicked(const QModelIndex &index) {
  const std::string model_name = m_ui->modelList->currentItem()->text().toStdString();
  LOG(INFO) << "model list clicked..." << index.row() << " - " << model_name;
  m_ui->add_model->setEnabled(true);
  updateModelWindow(model_name);
}

void MainWindow::on_add_model_clicked() {
  const std::string model_name = m_ui->modelList->currentItem()->text().toStdString();
  LOG(INFO) << "Adding object model " << model_name << " to hypotheses list.";
  addSelectedModelCloud(model_name);
}

void MainWindow::on_icp_button_clicked() {
  bool *okay = new bool();
  int icp_iter = m_ui->icp_iter_te->text().toInt(okay);
  if (!*okay) {
    LOG(ERROR) << "Could not convert icp iterations text field to a number. Is "
                  "it a number? Setting it to the default value "
               << 20;
    icp_iter = 20;
  }

  float icp_max_correspondence_distance = m_ui->icp_max_corresp_dist_te->text().toFloat(okay);
  if (!*okay) {
    LOG(ERROR) << "Could not convert icp iterations text field to a number. Is "
                  "it a number? Setting it to the default value "
               << 0.005;
    icp_iter = 0.005f;
  } else {
    icp_max_correspondence_distance /= 100.f;  // convert to meter
  }

  setStatus("Started ICP...");

  if (selected_hypothesis_ >= 0) {
    pcl::PointCloud<ModelT>::ConstPtr model_cloud = loaded_models_[hypotheses_[selected_hypothesis_].id_].cloud_;
    pcl::PointCloud<ModelT>::Ptr model_cloud_transformed(new pcl::PointCloud<ModelT>);
    pcl::transformPointCloud(*model_cloud, *model_cloud_transformed, hypotheses_[selected_hypothesis_].pose_);

    SceneT minPoint, maxPoint;
    pcl::getMinMax3D<ModelT>(*model_cloud_transformed, minPoint, maxPoint);
    float max_corr_distance = 0.1f;
    minPoint.x -= max_corr_distance;
    minPoint.y -= max_corr_distance;
    minPoint.z -= max_corr_distance;
    maxPoint.x += max_corr_distance;
    maxPoint.y += max_corr_distance;
    maxPoint.z += max_corr_distance;

    pcl::CropBox<SceneT> cropFilter;
    cropFilter.setInputCloud(scene_merged_cloud_);
    cropFilter.setMin(minPoint.getVector4fMap());
    cropFilter.setMax(maxPoint.getVector4fMap());
    pcl::PointCloud<SceneT>::Ptr scene_cropped_to_model(new pcl::PointCloud<SceneT>);
    cropFilter.filter(*scene_cropped_to_model);

    if (param_.icp_scene_to_model_) {
      pcl::IterativeClosestPoint<SceneT, ModelT> reg;
      reg.setMaximumIterations(icp_iter);
      reg.setEuclideanFitnessEpsilon(param_.icp_euclidean_fitness_eps_);
      reg.setTransformationEpsilon(param_.icp_transformation_eps_);
      reg.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
      reg.setInputTarget(model_cloud_transformed);
      reg.setInputSource(scene_cropped_to_model);
      pcl::PointCloud<ModelT> output;
      reg.align(output);
      hypotheses_[selected_hypothesis_].pose_ = reg.getFinalTransformation() * hypotheses_[selected_hypothesis_].pose_;
    } else {
      pcl::IterativeClosestPoint<ModelT, SceneT> reg;
      reg.setMaximumIterations(icp_iter);
      reg.setEuclideanFitnessEpsilon(param_.icp_euclidean_fitness_eps_);
      reg.setTransformationEpsilon(param_.icp_transformation_eps_);
      reg.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
      reg.setInputTarget(scene_cropped_to_model);
      reg.setInputSource(model_cloud_transformed);
      pcl::PointCloud<SceneT> output;
      reg.align(output);
      hypotheses_[selected_hypothesis_].pose_ = reg.getFinalTransformation() * hypotheses_[selected_hypothesis_].pose_;
    }

    updateSelectedHypothesis();
  }

  setStatus("ICP finished!");
}

void MainWindow::on_remove_clicked_pts_clicked() {
  clear_picked_model_points();
  clear_picked_scene_points();
}

size_t MainWindow::ObjectHypothesis::nextID = 0;
