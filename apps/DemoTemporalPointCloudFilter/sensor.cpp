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
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#ifndef Q_MOC_RUN
#include "sensor.h"
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/common/convertImage.h>
#include <v4r/features/FeatureDetector_KD_FAST_IMGD.h>
#endif

using namespace std;

/**
 * @brief Sensor::Sensor
 */
Sensor::Sensor()
: m_run(false), m_draw_mask(false), u_idle_time(15 * 1000),  // 15 ms
  max_queue_size(1),                                         // allow a queue size of 3 clouds
  ts(1) {
  // set cameras
  cam = cv::Mat_<double>::eye(3, 3);
  cam(0, 0) = 525;
  cam(1, 1) = 525;
  cam(0, 2) = 320;
  cam(1, 2) = 240;

  // set up tracking
  tsf.setCameraParameter(cam, 1.);
  param.tsf_filtering = true;
  param.tsf_mapping = false;
  param.pt_param.max_count = 250;
  param.pt_param.conf_tracked_points_norm = 150;
  param.map_param.refine_plk = false;
  param.map_param.detect_loops = true;
  param.map_param.nb_tracked_frames = 10;
  param.filt_param.batch_size_clouds = 5;  // batch size for smoothing
  param.diff_cam_distance_map = 0.5;
  param.diff_delta_angle_map = 7;
  param.filt_param.type = 3;  // 0...ori. col., 1..col mean, 2..bilin., 3..bilin col and cut off depth
  tsf.setParameter(param);

  sf_cloud.reset(new v4r::DataMatrix2D<v4r::Surfel>);

  v4r::FeatureDetector::Ptr detector(new v4r::FeatureDetector_KD_FAST_IMGD());
  tsf.setDetectors(detector, detector);
}

/**
 * @brief Sensor::~Sensor
 */
Sensor::~Sensor() {
  stop();
}

/******************************** public *******************************/

/**
 * @brief Sensor::setNumberOfSmoothedFrames
 * @param nb_frames
 */
void Sensor::setNumberOfSmoothedFrames(const int &nb_frames) {
  param.filt_param.batch_size_clouds = nb_frames;
  if (nb_frames >= 3)
    tsf.setParameter(param);
}

/**
 * @brief Sensor::start
 * @param cam_id
 */
void Sensor::start() {
  QThread::start();
}

/**
 * @brief Sensor::stop
 */
void Sensor::stop() {
  if (m_run) {
    m_run = false;
    this->wait();
  }
  tsf.stop();
}

/**
 * @brief Sensor::isRunning
 * @return
 */
bool Sensor::isRunning() {
  return m_run;
}

/**
 * @brief Sensor::reset
 */
void Sensor::reset() {}

/**
 * @brief Sensor::showDepthMask
 * @param _draw_depth_mask
 */
void Sensor::showDepthMask(bool _draw_depth_mask) {
  m_draw_mask = _draw_depth_mask;
}

/*************************************** private **************************************************/

void Sensor::CallbackCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud) {
  shm_mutex.lock();

  if (shm_clouds.size() < max_queue_size) {
    shm_clouds.push(pcl::PointCloud<pcl::PointXYZRGB>::Ptr());
    shm_clouds.back().reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*_cloud, *shm_clouds.back());
  }

  shm_mutex.unlock();

  usleep(u_idle_time);
}

/**
 * @brief Sensor::run
 * main loop
 */
void Sensor::run() {
  try {
    interface.reset(new pcl::OpenNIGrabber());
  } catch (pcl::IOException e) {
    m_run = false;
    emit printStatus(std::string("Status: No OpenNI device connected!"));
    return;
  }

  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f =
      boost::bind(&Sensor::CallbackCloud, this, _1);
  interface->registerCallback(f);
  interface->start();

  Eigen::Matrix4f pose;
  cv::Mat_<cv::Vec3b> im_draw;
  double conf_ransac_iter = 0, conf_tracked_points = 0;
  m_run = true;
  double ts_smoothed = 0, ts_smoothed_prev = 0;
  boost::posix_time::ptime time0 = boost::posix_time::microsec_clock::local_time();

  while (m_run) {
    // -------------------- do tracking --------------------------
    // get cloud
    shm_mutex.lock();

    if (!shm_clouds.empty()) {
      cloud = shm_clouds.front();
      shm_clouds.pop();
    } else
      cloud.reset();

    shm_mutex.unlock();

    if (cloud.get() != 0) {
      pcl::ScopeTime t("tracking");
      v4r::convertImage(*cloud, image);
      image.copyTo(im_draw);

      // track camera
      if (m_draw_mask)
        tsf.setDebugImage(im_draw);  // debug viz
      else
        tsf.setDebugImage(cv::Mat());

      ts = static_cast<double>((boost::posix_time::microsec_clock::local_time() - time0).total_milliseconds());
      tsf.track(*cloud, ts, pose, conf_ransac_iter, conf_tracked_points);

      // drawing
      if (m_draw_mask)
        drawDepthMask(*cloud, im_draw);
      drawConfidenceBar(im_draw, conf_tracked_points);
      emit new_image(im_draw, ts);

      if (param.filt_param.batch_size_clouds < 3) {
        emit new_cloud(cloud, ts);
      } else {
        tsf.getSurfelCloud(*sf_cloud, pose, ts_smoothed);
        if (ts_smoothed > ts_smoothed_prev) {
          cout << "timing smoothed frames: " << ts_smoothed - ts_smoothed_prev << " ms" << endl;

          ts_smoothed_prev = ts_smoothed;
          emit new_sf_cloud(sf_cloud, ts);
        }
      }
      emit update_visualization();
    } else
      usleep(u_idle_time / 2);
  }

  usleep(50000);
  interface->stop();
  usleep(50000);
}

/**
 * drawConfidenceBar
 */
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

/**
 * @brief Sensor::drawDepthMask
 * @param cloud
 * @param im
 */
void Sensor::drawDepthMask(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat &im) {
  if ((int)_cloud.width != im.cols || (int)_cloud.height != im.rows)
    return;

  for (unsigned i = 0; i < _cloud.width * _cloud.height; i++)
    if (isnan(_cloud.points[i].x))
      im.at<cv::Vec3b>(i) = cv::Vec3b(255, 0, 0);
}
