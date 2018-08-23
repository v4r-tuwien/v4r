/****************************************************************************
**
** Copyright (C) 2018 TU Wien, ACIN, Vision 4 Robotics (V4R) group
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
** For licensing terms and conditions please contact
** office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file apply_calibration.h
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */

#ifndef APPLY_CALIBRATION_H
#define APPLY_CALIBRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <v4r/calibration/calibration_params.h>
#include <v4r/core/macros.h>

namespace v4r {
namespace auto_calibration {

// Needed to convert double params to float. Float used for SSE optimized code.
struct V4R_EXPORTS fetch_calibration_params {
  std::array<float, 8> rgb_cam_int{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
  std::array<float, 8> depth_cam_int{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
  std::array<float, 2> depth_coeffs{{0.f, 0.f}};
  std::array<float, 6> Trel{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};

  Eigen::Matrix4f rot;

  void load_params(calibrationParams* params);
  void print_params();
};

/** @brief Apply Calibration.
 *         Class that holds methods to apply the calibration results from AutoCalibratuionFromSFM.
 *
 *  Generatea a (colored) PointCloud. See apply_calibration::GeneratePointCloud().
 *  Project corrected depth map to rgb image. See apply_calibration::GenerateDepthMap().
 *  Correct depth map (without projecting it). See apply_calibration::GenerateDepthMapNoProjection().
 */
class V4R_EXPORTS apply_calibration {
 public:
  /** @brief Constructor, unsing a caliibration file from the hard drive.
   *
   * calibration_file: *.ser file from calibration
   */
  apply_calibration(std::string& calibration_file);

  /** @brief Constructor, unsing a calibrationParams object.
   *
   */
  apply_calibration(calibrationParams* opt_params);

  /** @brief Generate Point Cloud
   *         Generates a corrected, organized, colored point cloud.
   *
   *  Generatea a (colored) PointCloud. See apply_calibration::GeneratePointCloud().
   *  Project corrected depth map to rgb image. See apply_calibration::GenerateDepthMap().
   *  Correct depth map (without projecting it). See apply_calibration::GenerateDepthMapNoProjection().
   */
  void GeneratePointCloud(const cv::Mat* depth, const cv::Mat* color, pcl::PointCloud<pcl::PointXYZRGB>& result);

  /** @brief Generate Point Cloud
   *         Generates a corrected, organized point cloud without clor information.
   *
   *
   */
  void GeneratePointCloud(const cv::Mat* depth, pcl::PointCloud<pcl::PointXYZ>& result);

  /** @brief Generate Depth Map
   *         Generates a corrected depth map and projects it to the uncalibrated rgb frame.
   *
   *
   */
  void GenerateDepthMap(const cv::Mat* depth, cv::Mat& result);

  /** @brief Generate Depth Map
   *         Generates a corrected depth map without projecting it to the rgb frame.
   *
   *
   */
  void GenerateDepthMapNoProjection(const cv::Mat* depth, cv::Mat& result);

 private:
  void generateBicubicLookupMat(calibrationParams* dparams);
  void generateExpLookup();
  void generateUndistortionLookup();

  int rgb_im_width_;
  int rgb_im_height_;
  int depth_im_width_;
  int depth_im_height_;
  fetch_calibration_params calib_params_;
  cv::Mat lattice_;
  cv::Mat depth_undistortion_;
  std::vector<float> exp_lookup_;
};
}  // namespace auto_calibration
}  // namespace v4r
#endif
