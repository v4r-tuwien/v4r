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

#ifndef KP_TEMPORAL_SMOOTHING_FILTER_RGBD_HH
#define KP_TEMPORAL_SMOOTHING_FILTER_RGBD_HH

#include <float.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/core/macros.h>
#include <v4r/keypoints/RigidTransformationRANSAC.h>
#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <v4r/common/impl/DataMatrix2D.hpp>

namespace v4r {

/**
 * TemporalSmoothingFilter
 */
class V4R_EXPORTS TemporalSmoothingFilter {
 public:
  /**
   * Parameter
   */
  class V4R_EXPORTS Parameter {
   public:
    cv::TermCriteria termcrit;
    cv::Size win_size;
    cv::Size subpix_win_size;
    int max_count;
    double pcent_reinit;
    int max_integration_frames;
    bool compute_normals;
    float scale_depth_cutoff;
    unsigned global_map_size;  /// 0 -> do not maintain a global map
    v4r::RigidTransformationRANSAC::Parameter rt;
    Parameter()
    : termcrit(cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03)), win_size(cv::Size(31, 31)),
      subpix_win_size(cv::Size(10, 10)), max_count(500), pcent_reinit(0.5), max_integration_frames(20),
      compute_normals(true), scale_depth_cutoff(0.03), global_map_size(0),
      rt(v4r::RigidTransformationRANSAC::Parameter(0.02, 0.01, 1500)) {}
  };
  /**
   * @brief The Surfel class
   */
  class V4R_EXPORTS Surfel {
   public:
    Eigen::Vector3f pt;
    Eigen::Vector3f n;
    float weight, norm;
    int r, g, b;
    bool is_stable;
    Surfel() : weight(0), is_stable(false) {}
    Surfel(const pcl::PointXYZRGB &_pt)
    : pt(_pt.getArray3fMap()), weight(0), r(_pt.r), g(_pt.g), b(_pt.b), is_stable(true) {
      if (!std::isnan(pt[0]) && !std::isnan(pt[1]) && !std::isnan(pt[2])) {
        n = pt.normalized();
        weight = 1.;
      }
    }
  };
  /**
   * Shared memory
   */
  class V4R_EXPORTS Shm {
   public:
    boost::mutex mtx_shm;

    bool need_init;
    int init_points;
    int lk_flags;
    cv::Mat gray, prev_gray;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    std::vector<cv::Point2f> points[2];
    std::vector<Eigen::Vector3f> points3d[2];
    Eigen::Matrix4f pose;     /// global pose of the current frame (cloud, gray, points[1], ....)
    Eigen::Matrix4f kf_pose;  /// pose of the keyframe (points[0], points3d[0], prev_gray

    inline void lock() {
      mtx_shm.lock();
    }
    inline void unlock() {
      mtx_shm.unlock();
    }
    void reset() {
      need_init = false;
      init_points = 0;
      lk_flags = 0;
      pose = Eigen::Matrix4f::Identity();
      kf_pose = Eigen::Matrix4f::Identity();
      prev_gray = cv::Mat();
    }

    Shm()
    : need_init(false), init_points(0), lk_flags(0), pose(Eigen::Matrix4f::Identity()),
      kf_pose(Eigen::Matrix4f::Identity()) {}
  };

 private:
  Parameter param;

  std::vector<cv::Vec4i> npat;

  cv::Mat_<double> intrinsic;

  std::vector<uchar> status;
  std::vector<float> err;
  cv::Mat image;
  std::vector<int> inliers;

  int im_width, im_height;
  int sf_width, sf_height;

  // ---- dbg draw ----
  // std::shared_ptr<TomGine::tgTomGineThread> tgwin2;
  // std::vector<Eigen::Vector3f> cams;
  // ---- end dbg ----

  bool run, have_thread;

  boost::thread th_obectmanagement;
  boost::thread th_init;

  Shm shm;

  //  Eigen::Matrix4f last_pose, pose, inc_pose;
  Eigen::Matrix4f sf_pose;
  v4r::DataMatrix2D<Surfel>::Ptr sf_cloud;
  v4r::DataMatrix2D<Surfel>::Ptr tmp_cloud;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr global_cloud;

  v4r::RigidTransformationRANSAC::Ptr rt;

  void operate();

  bool needReinit(const std::vector<cv::Point2f> &points);
  void getPoints3D(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::vector<cv::Point2f> &points,
                   std::vector<Eigen::Vector3f> &points3d);
  void filterValidPoints3D(std::vector<cv::Point2f> &points, std::vector<Eigen::Vector3f> &points3d);
  void filterValidPoints3D(std::vector<cv::Point2f> &pts1, std::vector<Eigen::Vector3f> &pts3d1,
                           std::vector<cv::Point2f> &pts2, std::vector<Eigen::Vector3f> &pts3d2);
  void filterInliers(std::vector<cv::Point2f> &pts1, std::vector<Eigen::Vector3f> &pts3d1,
                     std::vector<cv::Point2f> &pts2, std::vector<Eigen::Vector3f> &pts3d2, std::vector<int> &inliers);
  void addCloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose,
                v4r::DataMatrix2D<Surfel>::Ptr &sf_cloud, Eigen::Matrix4f &sf_pose);
  void convertSurfelMap(const v4r::DataMatrix2D<Surfel> &cfilt, pcl::PointCloud<pcl::PointXYZRGBNormal> &out);
  void logGlobalMap(const v4r::DataMatrix2D<Surfel> &cfilt, const Eigen::Matrix4f &pose,
                    pcl::PointCloud<pcl::PointXYZRGBNormal> &out);
  void computeNormal(v4r::DataMatrix2D<Surfel> &sf_cloud);

  inline bool intersectPlaneLine(const Eigen::Vector3f &p, const Eigen::Vector3f &n, const Eigen::Vector3f &r,
                                 Eigen::Vector3f &isct);

 public:
  cv::Mat dbg;

  TemporalSmoothingFilter(const Parameter &p = Parameter());
  ~TemporalSmoothingFilter();

  void start();
  void stop();

  inline bool isStarted() {
    return have_thread;
  }

  inline void lock() {
    shm.lock();
  }  // threaded object management, so we need to lock
  inline void unlock() {
    shm.unlock();
  }

  void reset();

  void filter(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &filtered_cloud,
              Eigen::Matrix4f &pose);

  void getGlobalCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_global_cloud);
  void setCameraParameter(const cv::Mat &_intrinsic);
  void setParameter(const Parameter &p) {
    param = p;
  }

  typedef std::shared_ptr<::v4r::TemporalSmoothingFilter> Ptr;
  typedef std::shared_ptr<::v4r::TemporalSmoothingFilter const> ConstPtr;
};

/*************************** INLINE METHODES **************************/
/**
 * compute plane ray intersection
 * the plane normal n and the ray r must be normalised
 */
inline bool TemporalSmoothingFilter::intersectPlaneLine(const Eigen::Vector3f &p, const Eigen::Vector3f &n,
                                                        const Eigen::Vector3f &r, Eigen::Vector3f &isct) {
  float tmp = n.dot(r);

  if (fabs(tmp) > std::numeric_limits<float>::epsilon()) {
    isct = (n.dot(p) / tmp) * r;
    return true;
  }

  return false;
}

}  // namespace v4r

#endif
