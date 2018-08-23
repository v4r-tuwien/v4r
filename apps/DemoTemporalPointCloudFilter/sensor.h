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

#ifndef _GRAB_PCD_SENSOR_H_
#define _GRAB_PCD_SENSOR_H_

#ifndef Q_MOC_RUN
#include <pcl/common/transforms.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/camera_tracking_and_mapping/TSFData.h>
#include <v4r/camera_tracking_and_mapping/TSFVisualSLAM.h>
#include <v4r/common/convertCloud.h>
#include <QMutex>
#include <QThread>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <v4r/camera_tracking_and_mapping/Surfel.hh>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#endif

class Sensor : public QThread {
  Q_OBJECT

 public:
  Sensor();
  ~Sensor();

  void start();
  void stop();
  bool isRunning();
  void reset();
  void showDepthMask(bool _draw_depth_mask);
  void setNumberOfSmoothedFrames(const int &nb_frames);

 public slots:

 signals:
  void new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const cv::Mat_<cv::Vec3b> &image,
                 const double &_ts);
  void new_image(const cv::Mat_<cv::Vec3b> &image, const double &_ts);
  void new_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const double &_ts);
  void new_sf_cloud(const v4r::DataMatrix2D<v4r::Surfel>::Ptr &_sf_cloud, const double &_ts);
  void update_visualization();
  void printStatus(const std::string &_txt);

 private:
  void run();

  void CallbackCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud);
  void drawConfidenceBar(cv::Mat &im, const double &conf);
  void drawDepthMask(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat &im);

  inline bool isNaN(const Eigen::Vector3f &pt);
  inline double sqr(const double &val);

  // status
  bool m_run;
  bool m_draw_mask;

  // parameter
  unsigned u_idle_time;
  unsigned max_queue_size;
  double ts;

  cv::Mat_<double> cam;

  v4r::TSFVisualSLAM::Parameter param;

  // camera tracker
  v4r::TSFVisualSLAM tsf;

  QMutex shm_mutex;
  std::queue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> shm_clouds;

  QMutex cloud_mutex;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  cv::Mat_<cv::Vec3b> image;
  v4r::DataMatrix2D<v4r::Surfel>::Ptr sf_cloud;

  std::shared_ptr<pcl::Grabber> interface;
};

/**
 * @brief Sensor::isNaN
 * @param pt
 * @return
 */
inline bool Sensor::isNaN(const Eigen::Vector3f &pt) {
  if (std::isnan(pt[0]) || std::isnan(pt[1]) || std::isnan(pt[2]))
    return true;
  return false;
}

inline double Sensor::sqr(const double &val) {
  return val * val;
}

#endif  // _GRAB_PCD_SENSOR_H_
