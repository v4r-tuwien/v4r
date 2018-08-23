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
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#ifndef Q_MOC_RUN
#include <QListView>
#include <QMainWindow>
#include <QObject>
#endif

#include <pcl/common/angles.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <v4r/common/intrinsics.h>
#include <boost/filesystem.hpp>
#include <boost/mpl/at.hpp>
#include <boost/mpl/map.hpp>

namespace bf = boost::filesystem;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

  typedef pcl::PointXYZRGB SceneT;  /// TODO Make this a template class
  typedef pcl::PointXYZRGB ModelT;  /// TODO Make this a template class

  typedef boost::mpl::map<boost::mpl::pair<pcl::PointXYZ, pcl::PointNormal>,
                          boost::mpl::pair<pcl::PointNormal, pcl::PointNormal>,
                          boost::mpl::pair<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>,
                          boost::mpl::pair<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>,
                          boost::mpl::pair<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>,
                          boost::mpl::pair<pcl::PointXYZI, pcl::PointXYZINormal>,
                          boost::mpl::pair<pcl::PointXYZINormal, pcl::PointXYZINormal>>
      PointTypeAssociations;
  BOOST_MPL_ASSERT((boost::mpl::has_key<PointTypeAssociations, ModelT>));

  typedef typename boost::mpl::at<PointTypeAssociations, ModelT>::type ModelTWithNormal;

 public:
  struct Parameter {
    float subtraction_radius_ = 0.01f;  ///< inlier threshold in meters for a point to be
    ///< subtracted if a nearby point within this
    ///< threshold exists in the background cloud

    bool icp_scene_to_model_ = false;
    float occlusion_threshold_ = 0.015f;
    float min_px_distance_to_image_boundary_ =
        3.f;  ///< minimum distance in pixel a re-projected point needs to have to the image boundary
    float min_dotproduct_model_normal_to_viewray_ =
        0.2f;  ///< surfaces which point are oriented away from the viewray will be
    /// discarded if the absolute dotproduct between the surface normal and
    /// the viewray is smaller than this threshold. This should ignore
    /// points for further fitness check which are very sensitive to small
    /// rotation changes.
    float icp_euclidean_fitness_eps_ = 1e-12;
    float icp_transformation_eps_ = 0.0001f * 0.0001f;
    float min_dist_to_keep_hypothesis_ =
        0.01f;  ///< minimum distance in meter to keep object hypothesis of the same object model when loaded from disk

    float voxel_grid_resolution_ = 0.005f;  ///< resolution in meter of downsampled registered scene cloud

    // visualization parameters
    int fontsize_ = 14;  ///< font size of the visualization windows
    bool visualize_occlusion_computation_ =
        false;  ///< if true, visualizes the visible points of the object hypotheses with respect to each view
  };

 private:
  class ObjectHypothesis {
   protected:
    std::string unique_id_;
    static size_t nextID;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string id_;        ///< object model name
    Eigen::Matrix4f pose_;  ///< object pose

    bool operator<(const ObjectHypothesis &h) const {
      return (id_ < h.id_);
    }

    ObjectHypothesis() {
      std::stringstream unique_id_ss;
      unique_id_ss << "hyp_" << nextID++;
      unique_id_ = unique_id_ss.str();
    }

    ObjectHypothesis(const ObjectHypothesis &orig) {
      unique_id_ = orig.unique_id_;
      id_ = orig.id_;
      pose_ = orig.pose_;
    }

    ObjectHypothesis &operator=(const ObjectHypothesis &orig) {
      unique_id_ = orig.unique_id_;
      id_ = orig.id_;
      pose_ = orig.pose_;
      return (*this);
    }

    std::string getUniqueId() const {
      return unique_id_;
    }
  };

  class View {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    pcl::PointCloud<SceneT>::Ptr cloud_;
    std::string filename_;
    Eigen::Matrix4f pose_;  // relative camera pose which aligns views when multiplied by this transformation matrix
  };

  class ObjectModel {
   public:
    pcl::PointCloud<ModelT>::ConstPtr cloud_;
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
    std::shared_ptr<pcl::octree::OctreePointCloudPointVector<ModelT>> octree_;
  };

  Ui::MainWindow *m_ui;

  Eigen::MatrixXi pt_colors_;  ///< unique point color for visualization of picked points

  void save_model() const;
  void updateMapListIndex2HypothesisId();
  void setupGUI();
  void clear_picked_scene_points();
  void clear_picked_model_points();

  pcl::visualization::PCLVisualizer::Ptr pviz_;
  pcl::visualization::PCLVisualizer::Ptr pviz_models_;

  enum class bg_color { BLACK, WHITE };

  bg_color bg_color_scene_ = bg_color::BLACK, bg_color_model_ = bg_color::BLACK;

  void pp_callback_scene(const pcl::visualization::PointPickingEvent &event);
  void pp_callback_model(const pcl::visualization::PointPickingEvent &event);
  void keyboard_callback_scene(const pcl::visualization::KeyboardEvent &event);
  void keyboard_callback_model(const pcl::visualization::KeyboardEvent &event);

  void next();
  void loadTestFiles();
  void prev();
  void enablePoseRefinmentButtons(bool flag);

  void enablePrevNextButtons();

  void updateSelectedHypothesis(bool keep_picked_pts = false);
  void updateImage(int state);
  float getTransStep() const;
  float getRotStep() const;

  void updatePose();
  void addSelectedModelCloud(const std::string &model_name,
                             const Eigen::Matrix4f &object_pose = Eigen::Matrix4f::Identity());
  void updateModelWindow(const std::string &model_name);
  void clear();
  void fillScene();
  void load_model();  ///< load annotated objects from disk if the annotation
                      ///< file already exists

  float getObjectVisibility(const ObjectHypothesis &h, const View &v);

  cv::Mat img_boundary_distance_;  ///< saves for each pixel how far it is away from the boundary (taking into account
  /// extrinsics of the camera)

  /**
   * @brief checks whether a given object hypothesis is already present
   * @return true if there is already a hpyothesis with the same object id and
   * pose (up to some threshold) is already present
   */
  bool isHypothesisRedundant(
      const std::vector<ObjectHypothesis, Eigen::aligned_allocator<ObjectHypothesis>> &existing_hypotheses,
      const ObjectHypothesis &h) const;

  void visPickedPoints();

  void setStatus(const std::string &info_text, bool append = false, int r = 0, int g = 0, int b = 0);

  pcl::PointCloud<SceneT>::Ptr scene_merged_cloud_;

  bf::path dir_models_;
  bf::path input_dir_;
  bf::path annotation_input_dir_;
  bf::path output_dir_;  ///< output directory where the annotation files will
  ///< be stored

  std::vector<View, Eigen::aligned_allocator<View>> scene_views_;

  std::vector<ObjectHypothesis, Eigen::aligned_allocator<ObjectHypothesis>> hypotheses_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr picked_model_pts_;  ///< clicked points defining model - scene
                                                             ///< correspondences

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr picked_scene_pts_;  ///< clicked points defining model - scene
                                                             ///< correspondences

  std::map<size_t, std::string> map_listId_2_hypotheses_id_;
  int selected_hypothesis_;
  std::map<std::string, ObjectModel> loaded_models_;
  std::vector<std::string> model_ids_;

  std::vector<std::string> test_sequences_;  ///< multi-view sequences
                                             ///< (single-view is also treated as
                                             ///< multi-view case)
  int current_sequence_id_;

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr bg_kdtree_;  ///< defines the background
                                                    ///< cloud that is subtracted
                                                    ///< from each individual
                                                    ///< cloud
  int vp_scene_1_, vp_scene_2_, vp_model_1_, vp_model_2_;

  v4r::Intrinsics cam_;
  Parameter param_;

 private Q_SLOTS:
  void on_x_plus_clicked();
  void on_x_minus_clicked();
  void on_y_plus_clicked();
  void on_y_minus_clicked();
  void on_z_plus_clicked();
  void on_z_minus_clicked();
  void on_save_model_clicked();
  void on_prev_clicked();
  void on_next_clicked();
  void on_xr_plus_clicked();
  void on_xr_minus_clicked();
  void on_yr_plus_clicked();
  void on_yr_minus_clicked();
  void on_zr_plus_clicked();
  void on_zr_minus_clicked();
  void on_remove_selected_hyp_clicked();
  void on_hypothesesList_clicked(const QModelIndex &index);
  void on_modelList_clicked(const QModelIndex &index);
  void on_icp_button_clicked();
  void on_remove_clicked_pts_clicked();
  void on_add_model_clicked();
  void on_blend_images_clicked();

 public:
  explicit MainWindow(int argc, char *argv[], const Parameter &p);
  virtual ~MainWindow() {}
};
