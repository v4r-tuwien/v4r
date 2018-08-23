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

#ifndef Q_MOC_RUN
#include "sensor.h"
#include <pcl/common/io.h>

#include <v4r/common/convertCloud.h>
#include <v4r/common/convertImage.h>
#include <v4r/common/unprojection.h>
#include <v4r/keypoints/RigidTransformationRANSAC.h>
#include <v4r/keypoints/impl/PoseIO.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/keypoints/impl/toString.hpp>
//#include "v4r/KeypointTools/ScopeTime.hpp"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#endif

using namespace std;

const double Sensor::OFFSET_BOUNDING_BOX_TRACKING = -0.02;

Sensor::Sensor()
: run_(false), run_tracker_(false), draw_depth_mas_(false), publish_camera_trajectory_(false), upscaling_(1.),
  roi_seed_x_(320), roi_seed_y_(280), roi_selected_(false), roi_activated_(false), frame_waiting_idle_time_(35 * 1000),
  camera_pose_(Eigen::Matrix4f::Identity()), base_transform_(Eigen::Matrix4f::Identity()), stream_uri_(""),
  bbox_scale_xy_(1.), bbox_scale_height_(1.), bbox_seg_offset_(0.01),
  bbox_min_(Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX)), bbox_max_(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX)),
  bbox_transform_(Eigen::Matrix4f::Identity()) {
  // set cameras
  camera_trajectory_.reset(new std::vector<CameraLocation>());

  v4r::ClusterNormalsToPlanes::Parameter p_param;
  p_param.thrAngle = 45;
  p_param.inlDist = 0.01;
  p_param.minPoints = 5000;
  p_param.least_squares_refinement = true;
  p_param.smooth_clustering = false;
  p_param.thrAngleSmooth = 30;
  p_param.inlDistSmooth = 0.01;
  p_param.minPointsSmooth = 3;
  plane_estimation_.reset(new v4r::ClusterNormalsToPlanes(p_param));

  v4r::ZAdaptiveNormals::Parameter n_param;
  n_param.adaptive = true;
  normal_estimation_.reset(new v4r::ZAdaptiveNormals(n_param));

  // set up tracking
  tsf_.setCameraParameter(camera_params_.getCameraMatrix(), upscaling_);
  tsf_params_.tsf_filtering = true;
  tsf_params_.tsf_mapping = true;
  tsf_params_.pt_param.max_count = 250;
  tsf_params_.pt_param.conf_tracked_points_norm = 150;
  tsf_params_.map_param.refine_plk = false;
  tsf_params_.map_param.detect_loops = true;
  tsf_params_.map_param.nb_tracked_frames = 10;
  tsf_params_.filt_param.batch_size_clouds = 20;
  tsf_params_.diff_cam_distance_map = 0.5;
  tsf_params_.diff_delta_angle_map = 7;
  tsf_params_.filt_param.type = 3;  // 0...ori. col., 1..col mean, 2..bilin., 3..bilin col and cut off depth
  tsf_.setParameter(tsf_params_);

  feature_detector_.reset(new v4r::FeatureDetector_KD_FAST_IMGD);
  tsf_.setDetectors(feature_detector_, feature_detector_);
}

Sensor::~Sensor() {
  stop();
}

/******************************** public *******************************/

void Sensor::setTSFParameter(int nb_tracked_frames_ba, int batch_size_clouds_tsf, const double &cam_dist_map,
                             const double &delta_angle_map) {
  tsf_params_.map_param.nb_tracked_frames = nb_tracked_frames_ba;
  tsf_params_.filt_param.batch_size_clouds = batch_size_clouds_tsf;
  tsf_params_.diff_cam_distance_map = cam_dist_map;
  tsf_params_.diff_delta_angle_map = delta_angle_map;
  tsf_.setParameter(tsf_params_);
}

// const std::shared_ptr< std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > >& Sensor::getClouds()
//{
//  stopTracker();
//}

void Sensor::start(const std::string &stream_uri) {
  stream_uri_ = stream_uri;
  QThread::start();
}

void Sensor::stop() {
  if (run_) {
    run_ = false;
    stopTracker();
    this->wait();
  }
  tsf_.stop();
}

void Sensor::startTracker() {
  if (!run_)
    start(stream_uri_);

  run_tracker_ = true;
}

void Sensor::stopTracker() {
  if (run_tracker_) {
    run_tracker_ = false;
  }
  tsf_.stop();
}

bool Sensor::isRunning() {
  return run_;
}

void Sensor::set_roi_params(const double &_bbox_scale_xy, const double &_bbox_scale_height, const double &_seg_offs) {
  bbox_scale_xy_ = _bbox_scale_xy;
  bbox_scale_height_ = _bbox_scale_height;
  bbox_seg_offset_ = _seg_offs;
}

void Sensor::select_roi(int x, int y) {
  roi_seed_x_ = x;
  roi_seed_y_ = y;
  roi_selected_ = true;
}

void Sensor::reset() {
  bool is_run_tracker = run_tracker_;

  stopTracker();

  tsf_.reset();

  camera_trajectory_.reset(new std::vector<CameraLocation>());

  camera_pose_ = Eigen::Matrix4f::Identity();

  if (is_run_tracker)
    startTracker();
}

void Sensor::showDepthMask(bool _draw_depth_mask) {
  draw_depth_mas_ = _draw_depth_mask;
}

void Sensor::selectROI(int _seed_x, int _seed_y) {
  roi_selected_ = true;
  roi_seed_x_ = _seed_x;
  roi_seed_y_ = _seed_y;
}

void Sensor::activateROI(int enable) {
  roi_activated_ = enable;
}

/*************************************** private **************************************************/

void Sensor::run() {
  grabber_ = v4r::io::createGrabber(stream_uri_);
  if (!grabber_) {
    run_ = false;
    run_tracker_ = false;
    emit printStatus(std::string("Status: unable to create grabber!"));
    return;
  }

  // Setup camera parameters
  camera_params_ = grabber_->getCameraIntrinsics();

  camera_trajectory_.reset(new std::vector<CameraLocation>());
  camera_pose_ = Eigen::Matrix4f::Identity();
  tsf_.setCameraParameter(camera_params_.getCameraMatrix(), upscaling_);

  emit cam_params_changed(camera_params_);

  run_ = true;

  bool have_pose = false;
  int z = 0;
  double conf_ransac_iter = 0, conf_tracked_points = 0;
  cv::Mat_<cv::Vec3b> im_draw;
  // uint64_t ts = INT_MAX, ts_last;
  Eigen::Matrix4f filt_pose;
  pcl::PointCloud<pcl::PointXYZRGBNormal> filt_cloud;
  cv::Mat color, depth;
  current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  while (run_) {
    // -------------------- do tracking --------------------------

    if (grabber_->hasMoreFrames()) {
      grabber_->grabFrame(color, depth);
      v4r::unproject(color, depth, grabber_->getCameraIntrinsics(), *current_cloud_);
      // v4r::ScopeTime t("tracking");
      v4r::convertCloud(*current_cloud_, current_kp_cloud_, current_image_);
      current_image_.copyTo(im_draw);

      // select a roi
      if (roi_selected_)
        detectROI(current_kp_cloud_);
      if (roi_activated_)
        maskCloud(bbox_transform_, bbox_min_, bbox_max_, *current_cloud_);

      // track camera
      if (run_tracker_) {
        //        if (draw_depth_mas_) tsf_.setDebugImage(im_draw); // debug viz
        //        else tsf_.setDebugImage(cv::Mat());

        have_pose = tsf_.track(*current_cloud_, z, camera_pose_, conf_ransac_iter, conf_tracked_points);

        // debug save
        //        tsf_.getFilteredCloudNormals(filt_cloud, filt_pose, ts);
        //        if (ts != ts_last && filt_cloud.points.size()>0)
        //        {
        //            pcl::io::savePCDFileBinary(std::string("log/")+v4r::toString(z)+std::string("-filt.pcd"),
        //            filt_cloud);
        //            convertImage(filt_cloud, current_image_);
        //            cv::imwrite(std::string("log/")+v4r::toString(z)+std::string("-filt.jpg"),current_image_);
        //            ts_last = ts;
        //        }
        if (have_pose) {
          Eigen::Matrix4f inv_pose;
          v4r::invPose(camera_pose_, inv_pose);
          camera_trajectory_->push_back(CameraLocation(z, 1, inv_pose.block<3, 1>(0, 3), inv_pose.block<3, 1>(0, 2)));
          if (publish_camera_trajectory_)
            emit update_cam_trajectory(camera_trajectory_);
        }
        if (roi_activated_) {
          bbox_transform_ = camera_pose_ * base_transform_;
          emit update_boundingbox(bbox_edges_, bbox_transform_);
        }
      }

      //      if (draw_depth_mas_) drawDepthMask(*current_cloud_,im_draw); // debug viz
      //      drawConfidenceBar(im_draw,conf_tracked_points);
      //      emit new_image(current_cloud_, im_draw);

      if (draw_depth_mas_)
        drawDepthMask(*current_cloud_, current_image_);
      drawConfidenceBar(current_image_, conf_tracked_points);
      emit new_image(current_cloud_, current_image_);

      emit update_visualization();

      z++;
    } else
      usleep(frame_waiting_idle_time_ / 2);
  }

  usleep(50000);
}

void Sensor::drawConfidenceBar(cv::Mat &im, const double &_conf) {
  int bar_start = 50, bar_end = 200;
  int diff = bar_end - bar_start;
  int draw_end = diff * _conf;
  double col_scale = 255. / (double)diff;
  cv::Point2f pt1(0, 30);
  cv::Point2f pt2(0, 30);
  cv::Vec3b col(0, 0, 0);

  if (draw_end <= 0)
    draw_end = 1;

  for (int i = 0; i < draw_end; i++) {
    col = cv::Vec3b(255 - (i * col_scale), i * col_scale, 0);
    pt1.x = bar_start + i;
    pt2.x = bar_start + i + 1;
    cv::line(im, pt1, pt2, CV_RGB(col[0], col[1], col[2]), 8);
  }
}

void Sensor::drawDepthMask(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat &im) {
  if ((int)_cloud.width != im.cols || (int)_cloud.height != im.rows)
    return;

  for (unsigned i = 0; i < _cloud.width * _cloud.height; i++)
    if (!pcl::isFinite(_cloud.points[i]))
      im.at<cv::Vec3b>(i) = cv::Vec3b(255, 0, 0);

  //  for (unsigned i=0; i<plane_.indices.size(); i++)
  //  {
  //    im.at<cv::Vec3b>(plane_.indices[i]) = cv::Vec3b(255,0,0);
  //  }
}

void Sensor::getInplaneTransform(const Eigen::Vector3f &pt, const Eigen::Vector3f &normal, Eigen::Matrix4f &_pose) {
  _pose.setIdentity();

  Eigen::Vector3f px, py;
  Eigen::Vector3f pz = normal;

  if (pt.dot(pz) > 0)
    pz *= -1;
  px = (Eigen::Vector3f(1, 0, 0).cross(pz)).normalized();
  py = (pz.cross(px)).normalized();

  _pose.block<3, 1>(0, 0) = px;
  _pose.block<3, 1>(0, 1) = py;
  _pose.block<3, 1>(0, 2) = pz;
  _pose.block<3, 1>(0, 3) = pt;
}

void Sensor::maskCloud(const Eigen::Matrix4f &_pose, const Eigen::Vector3f &_bb_min, const Eigen::Vector3f &_bb_max,
                       pcl::PointCloud<pcl::PointXYZRGB> &_cloud) {
  Eigen::Vector3f pt_glob;
  Eigen::Matrix4f inv_pose;
  v4r::invPose(_pose, inv_pose);
  Eigen::Matrix3f R = inv_pose.topLeftCorner<3, 3>();
  Eigen::Vector3f t = inv_pose.block<3, 1>(0, 3);

  for (unsigned i = 0; i < _cloud.points.size(); i++) {
    pcl::PointXYZRGB &pt = _cloud.points[i];

    if (pcl::isFinite(pt)) {
      pt_glob = R * pt.getVector3fMap() + t;

      if (pt_glob[0] < _bb_min[0] || pt_glob[0] > _bb_max[0] || pt_glob[1] < _bb_min[1] || pt_glob[1] > _bb_max[1] ||
          pt_glob[2] < _bb_min[2] || pt_glob[2] > _bb_max[2]) {
        pt.getVector3fMap() =
            Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN());
      }
    }
  }
}

void Sensor::getBoundingBox(const v4r::DataMatrix2D<Eigen::Vector3f> &_cloud, const std::vector<int> &_indices,
                            const Eigen::Matrix4f &_pose, std::vector<Eigen::Vector3f> &bbox, Eigen::Vector3f &_bb_min,
                            Eigen::Vector3f &_bb_max) {
  Eigen::Vector3f pt, bbox_center_xy;
  double xmin, xmax, ymin, ymax, bbox_height, h_bbox_length, h_bbox_width;

  xmin = ymin = DBL_MAX;
  xmax = ymax = -DBL_MAX;

  Eigen::Matrix4f inv_pose;
  v4r::invPose(_pose, inv_pose);
  Eigen::Matrix3f R = inv_pose.topLeftCorner<3, 3>();
  Eigen::Vector3f t = inv_pose.block<3, 1>(0, 3);

  for (unsigned i = 0; i < _indices.size(); i++) {
    pt = R * _cloud[_indices[i]] + t;
    if (pt[0] > xmax)
      xmax = pt[0];
    if (pt[0] < xmin)
      xmin = pt[0];
    if (pt[1] > ymax)
      ymax = pt[1];
    if (pt[1] < ymin)
      ymin = pt[1];
  }

  h_bbox_length = bbox_scale_xy_ * (xmax - xmin) / 2.;
  h_bbox_width = bbox_scale_xy_ * (ymax - ymin) / 2.;
  bbox_height = bbox_scale_height_ * (xmax - xmin + ymax - ymin) / 2.;
  bbox_center_xy = Eigen::Vector3f((xmin + xmax) / 2., (ymin + ymax) / 2., 0.);

  bbox.clear();
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, -h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, -h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, 0.));

  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, -h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, -h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, bbox_height));

  for (unsigned i = 0; i < 8; i += 2) {
    bbox.push_back(bbox[i]);
    bbox.push_back(bbox[i + 8]);
  }

  _bb_min = bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, 0);
  _bb_max = bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, bbox_height);
}

void Sensor::detectROI(const v4r::DataMatrix2D<Eigen::Vector3f> &_cloud) {
  v4r::DataMatrix2D<Eigen::Vector3f> normals;

  normal_estimation_->compute(_cloud, normals);
  plane_estimation_->compute(_cloud, normals, roi_seed_x_, roi_seed_y_, plane_);

  if (plane_.indices.size() > 3) {
    getInplaneTransform(plane_.pt, plane_.normal, bbox_transform_);
    getBoundingBox(_cloud, plane_.indices, bbox_transform_, bbox_edges_, bbox_min_, bbox_max_);
    base_transform_ = bbox_transform_;
    emit update_boundingbox(bbox_edges_, bbox_transform_);
    bbox_min_ += Eigen::Vector3f(0, 0, OFFSET_BOUNDING_BOX_TRACKING);
  }

  // cout<<"[Sensor::detectROI] roi plane nb pts: "<<plane_.indices.size()<<endl;

  roi_activated_ = true;
  roi_selected_ = false;
}

void Sensor::convertImage(const pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud, cv::Mat &_image) {
  _image = cv::Mat_<cv::Vec3b>(_cloud.height, _cloud.width);

  for (unsigned v = 0; v < _cloud.height; v++) {
    for (unsigned u = 0; u < _cloud.width; u++) {
      cv::Vec3b &cv_pt = _image.at<cv::Vec3b>(v, u);
      const pcl::PointXYZRGBNormal &pt = _cloud(u, v);

      cv_pt[2] = pt.r;
      cv_pt[1] = pt.g;
      cv_pt[0] = pt.b;
    }
  }
}
