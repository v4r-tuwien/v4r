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

#pragma once

#ifndef Q_MOC_RUN
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/camera_tracking_and_mapping/TSFData.h>
#include <v4r/camera_tracking_and_mapping/TSFGlobalCloudFilteringSimple.h>
#include <v4r/camera_tracking_and_mapping/TSFVisualSLAM.h>
#include <v4r/common/ZAdaptiveNormals.h>
#include <v4r/common/intrinsics.h>
#include <v4r/features/FeatureDetector_KD_FAST_IMGD.h>
#include <v4r/io/grabber.h>
#include <v4r/keypoints/ClusterNormalsToPlanes.h>
#include <QMutex>
#include <QThread>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/keypoints/impl/PoseIO.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/keypoints/impl/triple.hpp>
#endif

class Sensor : public QThread {
  Q_OBJECT

 public:
  class CameraLocation {
   public:
    int idx;
    int type;
    Eigen::Vector3f pt;
    Eigen::Vector3f vr;
    CameraLocation() {}
    CameraLocation(int _idx, int _type, const Eigen::Vector3f &_pt, const Eigen::Vector3f &_vr)
    : idx(_idx), type(_type), pt(_pt), vr(_vr) {}
  };

  Sensor();
  ~Sensor();

  void start(const std::string &stream_uri = "");
  void stop();
  void startTracker();
  void stopTracker();
  bool isRunning();
  void reset();
  void showDepthMask(bool _draw_depth_mask);
  void selectROI(int _seed_x, int _seed_y);
  void activateROI(int enable);
  void showCameras(int enable) {
    publish_camera_trajectory_ = enable;
  }
  void setTSFParameter(int nb_tracked_frames_ba, int batch_size_clouds_tsf, const double &cam_dist_map,
                       const double &delta_angle_map);

  void setVignettingCalibrationFiles(const std::string &vgn_file, const std::string &crf_file) {
    tsf_.setVignettingCalibrationFiles(vgn_file, crf_file);
  }

  inline const std::vector<v4r::TSFFrame::Ptr> &getMap() const {
    return tsf_.getMap();
  }
  void getObjectTransform(Eigen::Matrix4f &_base_transform, Eigen::Vector3f &_bb_min, Eigen::Vector3f &_bb_max) {
    _base_transform = base_transform_;
    _bb_min = bbox_min_ + Eigen::Vector3f(0, 0, -OFFSET_BOUNDING_BOX_TRACKING + bbox_seg_offset_);
    _bb_max = bbox_max_;
  }

  /// Get the current estimate of camera intrinsics (optimized by tracking subsystem).
  v4r::Intrinsics getOptimizedCameraIntrinsics() const {
    cv::Mat camera_matrix, distortion_coeffs;
    tsf_.getCameraParameter(camera_matrix, distortion_coeffs);
    return v4r::Intrinsics::fromCameraMatrixAndResolution(camera_matrix, camera_params_.w, camera_params_.h);
  }

  /// Get the current estimate of camera distortion coefficients (optimized by tracking subsystem).
  cv::Mat getOptimizedCameraDistortionCoefficients() const {
    cv::Mat camera_matrix, distortion_coeffs;
    tsf_.getCameraParameter(camera_matrix, distortion_coeffs);
    return distortion_coeffs;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public slots:
  void select_roi(int x, int y);
  void set_roi_params(const double &_bbox_scale_xy, const double &_bbox_scale_height, const double &_seg_offs);

 signals:
  void new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const cv::Mat_<cv::Vec3b> &image);
  void new_pose(const Eigen::Matrix4f &_pose);
  void update_model_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud);
  void update_cam_trajectory(const std::shared_ptr<std::vector<Sensor::CameraLocation>> &_cam_trajectory);
  void update_visualization();
  void printStatus(const std::string &_txt);
  void update_boundingbox(const std::vector<Eigen::Vector3f> &edges, const Eigen::Matrix4f &pose);
  void cam_params_changed(const v4r::Intrinsics &camera_params);

 private:
  /// Main thread loop, invoked by QThread.
  void run() override;

  void drawConfidenceBar(cv::Mat &im, const double &conf);
  void drawDepthMask(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat &im);
  void detectROI(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud);
  void getInplaneTransform(const Eigen::Vector3f &pt, const Eigen::Vector3f &normal, Eigen::Matrix4f &pose);
  void getBoundingBox(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, const std::vector<int> &indices,
                      const Eigen::Matrix4f &pose, std::vector<Eigen::Vector3f> &bbox, Eigen::Vector3f &bb_min,
                      Eigen::Vector3f &bb_max);
  void maskCloud(const Eigen::Matrix4f &pose, const Eigen::Vector3f &bb_min, const Eigen::Vector3f &bb_max,
                 pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

  void convertImage(const pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud, cv::Mat &_image);

  // status
  bool run_;
  bool run_tracker_;
  bool draw_depth_mas_;
  bool publish_camera_trajectory_;
  double upscaling_;

  // Modeling ROI
  int roi_seed_x_, roi_seed_y_;
  bool roi_selected_;
  bool roi_activated_;

  unsigned int frame_waiting_idle_time_;

  /// Estimated camera trajectory
  std::shared_ptr<std::vector<CameraLocation>> camera_trajectory_;

  // Camera tracking
  v4r::TSFVisualSLAM tsf_;
  v4r::TSFVisualSLAM::Parameter tsf_params_;
  v4r::FeatureDetector::Ptr feature_detector_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud_;
  v4r::DataMatrix2D<Eigen::Vector3f> current_kp_cloud_;
  cv::Mat_<cv::Vec3b> current_image_;

  Eigen::Matrix4f camera_pose_;
  Eigen::Matrix4f base_transform_;  ///< initial pose of the object (bounding box)

  v4r::io::Grabber::Ptr grabber_;
  std::string stream_uri_;

  // Bounding box filter
  static const double OFFSET_BOUNDING_BOX_TRACKING;
  double bbox_scale_xy_, bbox_scale_height_, bbox_seg_offset_;
  Eigen::Vector3f bbox_min_, bbox_max_;
  std::vector<Eigen::Vector3f> bbox_edges_;
  Eigen::Matrix4f bbox_transform_;

  v4r::ClusterNormalsToPlanes::Ptr plane_estimation_;
  v4r::ZAdaptiveNormals::Ptr normal_estimation_;
  v4r::ClusterNormalsToPlanes::Plane plane_;

  /// Camera intrinsic parameters
  v4r::Intrinsics camera_params_;
};
