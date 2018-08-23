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
 * @file apply_calibration.cpp
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */
#include <fstream>

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <opencv2/imgproc/imgproc.hpp>

#include <v4r/calibration/apply_calibration.h>
#include <v4r/common/bicubic_interpolation.h>

using namespace v4r::auto_calibration;

void fetch_calibration_params::print_params() {
  LOG(INFO) << "a0,a1: " << depth_coeffs[0] << "\t" << depth_coeffs[1] << std::endl;

  LOG(INFO) << "depth cam intrinsics: ";
  for (size_t i = 0; i < 8; i++) {
    LOG(INFO) << "\t" << depth_cam_int[i];
  }
  LOG(INFO) << std::endl;

  LOG(INFO) << "rgb cam intrinsics: ";
  for (size_t i = 0; i < 8; i++) {
    LOG(INFO) << "\t" << rgb_cam_int[i];
  }
  LOG(INFO) << std::endl;

  LOG(INFO) << "Trel: ";
  for (size_t i = 0; i < 6; i++) {
    LOG(INFO) << "\t" << Trel[i];
  }
  LOG(INFO) << std::endl;
}

void fetch_calibration_params::load_params(calibrationParams* params) {
  double a[2];
  Eigen::Matrix4f rot_temp4;

  params->getAParams(&a[0]);
  depth_coeffs[0] = a[0];
  depth_coeffs[1] = a[1];

  double depth_int[8];
  params->getDepthParams(&depth_int[0]);
  for (size_t i = 0; i < 8; i++) {
    depth_cam_int[i] = depth_int[i];
  }

  double rgb_int[8];
  params->getRGBParams(&rgb_int[0]);
  for (size_t i = 0; i < 8; i++) {
    rgb_cam_int[i] = rgb_int[i];
  }

  double Treld[6];
  params->getTrelParams(&Treld[0]);
  for (size_t i = 0; i < 6; i++) {
    Trel[i] = Treld[i];
  }

  Eigen::Vector3f rodr;
  rodr[0] = Trel[0];
  rodr[1] = Trel[1];
  rodr[2] = Trel[2];

  Eigen::AngleAxisf angle_axis(rodr.norm(), rodr.normalized());

  Eigen::Matrix3f rot_temp;
  rot_temp = angle_axis.toRotationMatrix();
  rot << rot_temp(0, 0), rot_temp(0, 1), rot_temp(0, 2), Trel[3], rot_temp(1, 0), rot_temp(1, 1), rot_temp(1, 2),
      Trel[4], rot_temp(2, 0), rot_temp(2, 1), rot_temp(2, 2), Trel[5], 0, 0, 0, 1;

  rot_temp4 = rot.inverse();
  rot = rot_temp4;
}

apply_calibration::apply_calibration(std::string& calibration_file) {
  // load calibration file
  calibrationParams dparams(0, 0);
  std::ifstream ifs(calibration_file.c_str());

  boost::archive::text_iarchive ia(ifs);
  try {
    LOG(INFO) << "loading.." << std::endl;
    ia >> dparams;
  } catch (const std::exception& e) {
    LOG(INFO) << "EXCEPTION:" << e.what();
  }

  calib_params_.load_params(&dparams);

  // Print loaded parameters
  calib_params_.print_params();

  dparams.getRGBImgSize(rgb_im_height_, rgb_im_width_);
  dparams.getDepthImgSize(depth_im_height_, depth_im_width_);

  // Generate Lookup Tables
  generateBicubicLookupMat(&dparams);
  generateUndistortionLookup();
  generateExpLookup();
}

apply_calibration::apply_calibration(calibrationParams* opt_params) {
  // load calibration file

  calib_params_.load_params(opt_params);
  // Print loaded parameters
  calib_params_.print_params();

  // Generate Lookup Tables
  generateBicubicLookupMat(opt_params);
  generateUndistortionLookup();
  generateExpLookup();
}

void apply_calibration::generateUndistortionLookup() {
  // Precompute depth undistortion

  depth_undistortion_ = cv::Mat::zeros(1, depth_im_height_ * depth_im_width_, CV_32FC2);

  cv::Mat points(1, depth_im_height_ * depth_im_width_, CV_32FC2);
  cv::Vec4f coeffs;
  cv::Mat camera_matrix(3, 3, CV_32FC1);

  for (int row = 0; row < depth_im_height_; ++row) {
    for (int col = 0; col < depth_im_width_; ++col) {
      points.at<cv::Vec2f>(0, col + row * depth_im_width_)[0] = col;
      points.at<cv::Vec2f>(0, col + row * depth_im_width_)[1] = row;
    }
  }

  float dfx = calib_params_.depth_cam_int[4];  // fx
  float dfy = calib_params_.depth_cam_int[5];  // fy
  float dpx = calib_params_.depth_cam_int[6];  // px
  float dpy = calib_params_.depth_cam_int[7];  // py

  camera_matrix.at<float>(0, 0) = dfx;
  camera_matrix.at<float>(0, 1) = 0.0;
  camera_matrix.at<float>(0, 2) = dpx;
  camera_matrix.at<float>(1, 0) = 0.0;
  camera_matrix.at<float>(1, 1) = dfy;
  camera_matrix.at<float>(1, 2) = dpy;
  camera_matrix.at<float>(2, 0) = 0.0;
  camera_matrix.at<float>(2, 1) = 0.0;
  camera_matrix.at<float>(2, 2) = 1.0;

  // camera_parameter k1 ..k4;
  for (size_t i = 0; i < 4; i++) {
    coeffs[i] = calib_params_.depth_cam_int[i];
  }

  cv::undistortPoints(points, depth_undistortion_, camera_matrix, coeffs);
  depth_undistortion_ = depth_undistortion_.reshape(2, depth_im_height_);
}

void apply_calibration::generateExpLookup() {
  for (size_t depth = 0; depth < 20000; depth++) {
    exp_lookup_.push_back(exp(calib_params_.depth_coeffs[0] - calib_params_.depth_coeffs[1] * (1000.0 / float(depth))));
  }
}

void apply_calibration::generateBicubicLookupMat(calibrationParams* dparams) {
  cv::Size im_size(depth_im_width_, depth_im_height_);

  // Fill bicubic interpolation
  lattice_ = cv::Mat::zeros(im_size, CV_32FC1);

  int lattice_h, lattice_w;
  dparams->getLatticeSize(lattice_h, lattice_w);
  cv::Size size_lattice(lattice_w, lattice_h);
  const std::vector<double>& lattice_vector = dparams->getLatticeParams();

  std::vector<int> indices(16);
  Eigen::VectorXd betas(16);

  for (int u_y = 0; u_y < depth_im_height_; u_y++) {
    for (int u_x = 0; u_x < depth_im_width_; u_x++) {
      float sample_interpol = v4r::BicubicInterpolation::interpolatePoint(u_x, u_y, &lattice_vector[0], size_lattice,
                                                                          im_size, indices, betas);
      lattice_.at<float>(u_y, u_x) = sample_interpol;
    }
  }
}

void apply_calibration::GeneratePointCloud(const cv::Mat* depth, pcl::PointCloud<pcl::PointXYZ>& result) {
// SSE Implementation of xyz - point generation
#pragma omp parallel for
  for (int rows = 0; rows < depth->rows; rows++) {
    for (int cols = 0; cols < depth->cols; cols = cols + 4) {
      float depth_sample[4];

      cv::Vec2f uv_undist_cv[4];

      for (unsigned int i = 0; i < 4; i++) {
        depth_sample[i] = depth->at<float>(rows, cols + i);

        uv_undist_cv[i] = depth_undistortion_.at<cv::Vec2f>(rows, cols + i);
      }

      // Project uv,depth to 3D space, project to rgb, using scale TREL and
      // intrinsics. make a color lookup (maybe an interpolation).

      float depth_sample_[]
          __attribute__((aligned(16))) = {depth_sample[0], depth_sample[1], depth_sample[2], depth_sample[3]};

      float uv_x[] __attribute__((aligned(16))) = {uv_undist_cv[0][0], uv_undist_cv[1][0], uv_undist_cv[2][0],
                                                   uv_undist_cv[3][0]};
      float uv_y[] __attribute__((aligned(16))) = {uv_undist_cv[0][1], uv_undist_cv[1][1], uv_undist_cv[2][1],
                                                   uv_undist_cv[3][1]};

      __m128* depth_measure = (__m128*)depth_sample_;

      __m128* uv_x_ptr = (__m128*)uv_x;
      __m128* uv_y_ptr = (__m128*)uv_y;

      __m128 x_ = _mm_mul_ps(*uv_x_ptr, *depth_measure);
      __m128 y_ = _mm_mul_ps(*uv_y_ptr, *depth_measure);

      float x[4];
      float y[4];
      float z[4];

      _mm_storeu_ps(&x[0], x_);
      _mm_storeu_ps(&y[0], y_);
      _mm_storeu_ps(&z[0], *depth_measure);

      pcl::PointXYZ point[4];
      for (int i = 0; i < 4; i++) {
        if (depth_sample[i] == 0.0) {
          point[i].x = 0.;
          point[i].y = 0.;
          point[i].z = 0.;

          result.points[depth->cols * rows + cols + i] = point[i];
          continue;
        } else {
          point[i].x = x[i];
          point[i].y = y[i];
          point[i].z = z[i];

          result.points[depth->cols * rows + cols + i] = point[i];
        }
      }
    }
  }
}

void apply_calibration::GenerateDepthMap(const cv::Mat* depth, cv::Mat& result) {
  pcl::PointCloud<pcl::PointXYZRGB> temp(depth->cols, depth->rows);
  pcl::PointCloud<pcl::PointXYZRGB> cloud(depth->cols, depth->rows);

  float k1 = calib_params_.rgb_cam_int[0];
  float k2 = calib_params_.rgb_cam_int[1];
  float k3 = calib_params_.rgb_cam_int[2];
  float k4 = calib_params_.rgb_cam_int[3];
  float fx = calib_params_.rgb_cam_int[4];
  float fy = calib_params_.rgb_cam_int[5];
  float px = calib_params_.rgb_cam_int[6];
  float py = calib_params_.rgb_cam_int[7];

// SSE Implementation of xyz - point generation
#pragma omp parallel for
  for (int rows = 0; rows < depth->rows; rows++) {
    for (int cols = 0; cols < depth->cols; cols = cols + 4) {
      float depth_sample[4];
      float ex[4];
      float sample_interpol[4];

      cv::Vec2f uv_undist_cv[4];

      for (unsigned int i = 0; i < 4; i++) {
        depth_sample[i] = depth->at<float>(rows, cols + i);
        ex[i] = exp_lookup_[unsigned(depth_sample[i] * 1000.)];
        uv_undist_cv[i] = depth_undistortion_.at<cv::Vec2f>(rows, cols + i);
        sample_interpol[i] = lattice_.at<float>(rows, cols + i);
      }

      // Project uv,depth to 3D space, project to rgb, using scale TREL and
      // intrinsics. make a color lookup (maybe an interpolation).

      float depth_sample_[]
          __attribute__((aligned(16))) = {depth_sample[0], depth_sample[1], depth_sample[2], depth_sample[3]};
      float sample_int[] __attribute__((aligned(16))) = {sample_interpol[0], sample_interpol[1], sample_interpol[2],
                                                         sample_interpol[3]};
      float ex_[] __attribute__((aligned(16))) = {ex[0], ex[1], ex[2], ex[3]};
      float uv_x[] __attribute__((aligned(16))) = {uv_undist_cv[0][0], uv_undist_cv[1][0], uv_undist_cv[2][0],
                                                   uv_undist_cv[3][0]};
      float uv_y[] __attribute__((aligned(16))) = {uv_undist_cv[0][1], uv_undist_cv[1][1], uv_undist_cv[2][1],
                                                   uv_undist_cv[3][1]};

      __m128* depth_ptr = (__m128*)depth_sample_;
      __m128* ex_ptr = (__m128*)ex_;
      __m128* uv_x_ptr = (__m128*)uv_x;
      __m128* uv_y_ptr = (__m128*)uv_y;
      __m128* sample_int_ptr = (__m128*)sample_int;

      __m128 invDepth = _mm_rcp_ps(*depth_ptr);
      __m128 delta = _mm_mul_ps(*ex_ptr, *sample_int_ptr);
      __m128 depth_measure = _mm_rcp_ps(_mm_add_ps(invDepth, delta));

      __m128 x_ = _mm_mul_ps(*uv_x_ptr, depth_measure);
      __m128 y_ = _mm_mul_ps(*uv_y_ptr, depth_measure);

      float x[4];
      float y[4];
      float z[4];

      _mm_storeu_ps(&x[0], x_);
      _mm_storeu_ps(&y[0], y_);
      _mm_storeu_ps(&z[0], depth_measure);

      pcl::PointXYZRGB point[4];
      for (int i = 0; i < 4; i++) {
        if (depth_sample[i] == 0.0) {
          point[i].x = 0;
          point[i].y = 0;
          point[i].z = 0;
          point[i].r = uint8_t(0);
          point[i].g = uint8_t(0);
          point[i].b = uint8_t(0);

          cloud.points[depth->cols * rows + cols + i] = point[i];
          continue;
        } else {
          point[i].x = x[i];
          point[i].y = y[i];
          point[i].z = z[i];

          cloud.points[depth->cols * rows + cols + i] = point[i];
        }
      }
    }
  }

  pcl::transformPointCloud(cloud, temp, calib_params_.rot);

  float k1_[] __attribute__((aligned(16))) = {k1, k1, k1, k1};
  float k2_[] __attribute__((aligned(16))) = {k2, k2, k2, k2};
  float k3_[] __attribute__((aligned(16))) = {k3, k3, k3, k3};
  float k4_[] __attribute__((aligned(16))) = {k4, k4, k4, k4};

  float one[] __attribute__((aligned(16))) = {1.0, 1.0, 1.0, 1.0};

  float fx_[] __attribute__((aligned(16))) = {fx, fx, fx, fx};
  float fy_[] __attribute__((aligned(16))) = {fy, fy, fy, fy};
  float px_[] __attribute__((aligned(16))) = {px, px, px, px};
  float py_[] __attribute__((aligned(16))) = {py, py, py, py};

  __m128* k1_ptr = (__m128*)k1_;
  __m128* k2_ptr = (__m128*)k2_;
  __m128* k3_ptr = (__m128*)k3_;
  __m128* k4_ptr = (__m128*)k4_;

  __m128* one_ptr = (__m128*)one;

  __m128* fx_ptr = (__m128*)fx_;
  __m128* fy_ptr = (__m128*)fy_;
  __m128* px_ptr = (__m128*)px_;
  __m128* py_ptr = (__m128*)py_;

#pragma omp parallel for
  for (int rows = 0; rows < depth->rows; rows++) {
    for (int cols = 0; cols < depth->cols; cols = cols + 4) {
      // load next 4 points
      pcl::PointXYZRGB point[4];
      point[0] = temp.points[depth->cols * rows + cols + 0];
      point[1] = temp.points[depth->cols * rows + cols + 1];
      point[2] = temp.points[depth->cols * rows + cols + 2];
      point[3] = temp.points[depth->cols * rows + cols + 3];

      // SSE implemented Projection to camera

      float x[] __attribute__((aligned(16))) = {point[0].x, point[1].x, point[2].x, point[3].x};
      float y[] __attribute__((aligned(16))) = {point[0].y, point[1].y, point[2].y, point[3].y};
      float z[] __attribute__((aligned(16))) = {point[0].z, point[1].z, point[2].z, point[3].z};

      __m128* x_ptr = (__m128*)x;
      __m128* y_ptr = (__m128*)y;
      __m128* z_ptr = (__m128*)z;

      float ux_f[4];
      float uy_f[4];

      __m128 re_z = _mm_rcp_ps(*z_ptr);
      __m128 xn = _mm_mul_ps(*x_ptr, re_z);
      __m128 yn = _mm_mul_ps(*y_ptr, re_z);

      __m128 xn2 = _mm_mul_ps(xn, xn);
      __m128 yn2 = _mm_mul_ps(yn, yn);

      __m128 r2 = _mm_add_ps(xn2, yn2);
      __m128 r22 = _mm_mul_ps(r2, r2);

      __m128 k2_r22 = _mm_mul_ps(*k2_ptr, r22);
      __m128 k1_r2 = _mm_mul_ps(*k1_ptr, r2);

      __m128 dist = _mm_add_ps(_mm_add_ps(*one_ptr, k1_r2), k2_r22);

      __m128 dist_xn = _mm_mul_ps(dist, xn);

      __m128 xnyn = _mm_mul_ps(xn, yn);
      __m128 _2xn2 = _mm_add_ps(xn2, xn2);
      __m128 _2xnyn = _mm_add_ps(xnyn, xnyn);
      __m128 _2yn2 = _mm_add_ps(yn2, yn2);

      __m128 dist_yn = _mm_mul_ps(dist, yn);

      __m128 r2_2xn2 = _mm_add_ps(r2, _2xn2);
      __m128 r2_2yn2 = _mm_add_ps(r2, _2yn2);

      __m128 k3_2xnyn = _mm_mul_ps(*k3_ptr, _2xnyn);
      __m128 k3_r2_2yn2 = _mm_mul_ps(*k3_ptr, r2_2yn2);

      __m128 k4_r2_2xn2 = _mm_mul_ps(*k4_ptr, r2_2xn2);
      __m128 k4_2xnyn = _mm_mul_ps(*k4_ptr, _2xnyn);

      __m128 xd = _mm_add_ps(dist_xn, k3_2xnyn);
      xd = _mm_add_ps(xd, k4_r2_2xn2);

      __m128 yd = _mm_add_ps(dist_yn, k3_r2_2yn2);
      yd = _mm_add_ps(yd, k4_2xnyn);

      __m128 fx_xd = _mm_mul_ps(*fx_ptr, xd);
      __m128 fy_yd = _mm_mul_ps(*fy_ptr, yd);

      __m128 ux = _mm_add_ps(fx_xd, *px_ptr);
      __m128 uy = _mm_add_ps(fy_yd, *py_ptr);

      _mm_storeu_ps(&ux_f[0], ux);
      _mm_storeu_ps(&uy_f[0], uy);

      // Store results in opencv image
      for (unsigned int i = 0; i < 4; i++) {
        if (ux_f[i] >= float(result.cols) || (ux_f[i]) < 0.0 || point[i].z == 0 || std::isnan(ux_f[i])) {
          continue;
        }  // if point[i] projects out of rgb image, go to next point[i].

        if (uy_f[i] >= float(result.rows) || uy_f[i] < 0.0 || std::isnan(uy_f[i])) {
          continue;
        }  // if point[i] projects out of rgb image, go to next point[i].

        result.at<ushort>(int(uy_f[i]), int(ux_f[i])) = ushort(point[i].z * 1000.);
      }
    }
  }

  // cv::imshow("ColorMap",*color);
  // cv::waitKey(10);
}

void apply_calibration::GeneratePointCloud(const cv::Mat* depth, const cv::Mat* color,
                                           pcl::PointCloud<pcl::PointXYZRGB>& result) {
  pcl::PointCloud<pcl::PointXYZRGB> temp(depth->cols, depth->rows);

  float k1 = calib_params_.rgb_cam_int[0];
  float k2 = calib_params_.rgb_cam_int[1];
  float k3 = calib_params_.rgb_cam_int[2];
  float k4 = calib_params_.rgb_cam_int[3];
  float fx = calib_params_.rgb_cam_int[4];
  float fy = calib_params_.rgb_cam_int[5];
  float px = calib_params_.rgb_cam_int[6];
  float py = calib_params_.rgb_cam_int[7];

// SSE Implementation of xyz - point generation
#pragma omp parallel for
  for (int rows = 0; rows < depth->rows; rows++) {
    for (int cols = 0; cols < depth->cols; cols = cols + 4) {
      float depth_sample[4];
      float ex[4];
      float sample_interpol[4];

      cv::Vec2f uv_undist_cv[4];

      for (unsigned int i = 0; i < 4; i++) {
        depth_sample[i] = depth->at<float>(rows, cols + i);
        ex[i] = exp_lookup_[unsigned(depth_sample[i] * 1000)];
        uv_undist_cv[i] = depth_undistortion_.at<cv::Vec2f>(rows, cols + i);
        sample_interpol[i] = lattice_.at<float>(rows, cols + i);
      }

      // Project uv,depth to 3D space, project to rgb, using scale TREL and
      // intrinsics. make a color lookup (maybe an interpolation).

      float depth_sample_[]
          __attribute__((aligned(16))) = {depth_sample[0], depth_sample[1], depth_sample[2], depth_sample[3]};
      float sample_int[] __attribute__((aligned(16))) = {sample_interpol[0], sample_interpol[1], sample_interpol[2],
                                                         sample_interpol[3]};
      float ex_[] __attribute__((aligned(16))) = {ex[0], ex[1], ex[2], ex[3]};
      float uv_x[] __attribute__((aligned(16))) = {uv_undist_cv[0][0], uv_undist_cv[1][0], uv_undist_cv[2][0],
                                                   uv_undist_cv[3][0]};
      float uv_y[] __attribute__((aligned(16))) = {uv_undist_cv[0][1], uv_undist_cv[1][1], uv_undist_cv[2][1],
                                                   uv_undist_cv[3][1]};

      __m128* depth_ptr = (__m128*)depth_sample_;
      __m128* ex_ptr = (__m128*)ex_;
      __m128* uv_x_ptr = (__m128*)uv_x;
      __m128* uv_y_ptr = (__m128*)uv_y;
      __m128* sample_int_ptr = (__m128*)sample_int;

      __m128 invDepth = _mm_rcp_ps(*depth_ptr);
      __m128 delta = _mm_mul_ps(*ex_ptr, *sample_int_ptr);
      __m128 depth_measure = _mm_rcp_ps(_mm_add_ps(invDepth, delta));

      __m128 x_ = _mm_mul_ps(*uv_x_ptr, depth_measure);
      __m128 y_ = _mm_mul_ps(*uv_y_ptr, depth_measure);

      float x[4];
      float y[4];
      float z[4];

      _mm_storeu_ps(&x[0], x_);
      _mm_storeu_ps(&y[0], y_);
      _mm_storeu_ps(&z[0], depth_measure);

      pcl::PointXYZRGB point[4];
      for (int i = 0; i < 4; i++) {
        if (depth_sample[i] == 0.0) {
          point[i].x = 0;
          point[i].y = 0;
          point[i].z = 0;
          point[i].r = uint8_t(0);
          point[i].g = uint8_t(0);
          point[i].b = uint8_t(0);

          result.points[depth->cols * rows + cols + i] = point[i];
          continue;
        } else {
          point[i].x = x[i];
          point[i].y = y[i];
          point[i].z = z[i];

          result.points[depth->cols * rows + cols + i] = point[i];
        }
      }
    }
  }

  pcl::transformPointCloud(result, temp, calib_params_.rot);

  float k1_[] __attribute__((aligned(16))) = {k1, k1, k1, k1};
  float k2_[] __attribute__((aligned(16))) = {k2, k2, k2, k2};
  float k3_[] __attribute__((aligned(16))) = {k3, k3, k3, k3};
  float k4_[] __attribute__((aligned(16))) = {k4, k4, k4, k4};

  float one[] __attribute__((aligned(16))) = {1.0, 1.0, 1.0, 1.0};

  float fx_[] __attribute__((aligned(16))) = {fx, fx, fx, fx};
  float fy_[] __attribute__((aligned(16))) = {fy, fy, fy, fy};
  float px_[] __attribute__((aligned(16))) = {px, px, px, px};
  float py_[] __attribute__((aligned(16))) = {py, py, py, py};

  __m128* k1_ptr = (__m128*)k1_;
  __m128* k2_ptr = (__m128*)k2_;
  __m128* k3_ptr = (__m128*)k3_;
  __m128* k4_ptr = (__m128*)k4_;

  __m128* one_ptr = (__m128*)one;

  __m128* fx_ptr = (__m128*)fx_;
  __m128* fy_ptr = (__m128*)fy_;
  __m128* px_ptr = (__m128*)px_;
  __m128* py_ptr = (__m128*)py_;

#pragma omp parallel for
  for (int rows = 0; rows < depth->rows; rows++) {
    for (int cols = 0; cols < depth->cols; cols = cols + 4) {
      // load next 4 points
      pcl::PointXYZRGB point[4];
      point[0] = temp.points[depth->cols * rows + cols + 0];
      point[1] = temp.points[depth->cols * rows + cols + 1];
      point[2] = temp.points[depth->cols * rows + cols + 2];
      point[3] = temp.points[depth->cols * rows + cols + 3];

      // SSE implemented Projection to camera

      float x[] __attribute__((aligned(16))) = {point[0].x, point[1].x, point[2].x, point[3].x};
      float y[] __attribute__((aligned(16))) = {point[0].y, point[1].y, point[2].y, point[3].y};
      float z[] __attribute__((aligned(16))) = {point[0].z, point[1].z, point[2].z, point[3].z};

      __m128* x_ptr = (__m128*)x;
      __m128* y_ptr = (__m128*)y;
      __m128* z_ptr = (__m128*)z;

      float ux_f[4];
      float uy_f[4];

      __m128 re_z = _mm_rcp_ps(*z_ptr);
      __m128 xn = _mm_mul_ps(*x_ptr, re_z);
      __m128 yn = _mm_mul_ps(*y_ptr, re_z);

      __m128 xn2 = _mm_mul_ps(xn, xn);
      __m128 yn2 = _mm_mul_ps(yn, yn);

      __m128 r2 = _mm_add_ps(xn2, yn2);
      __m128 r22 = _mm_mul_ps(r2, r2);

      __m128 k2_r22 = _mm_mul_ps(*k2_ptr, r22);
      __m128 k1_r2 = _mm_mul_ps(*k1_ptr, r2);

      __m128 dist = _mm_add_ps(_mm_add_ps(*one_ptr, k1_r2), k2_r22);

      __m128 dist_xn = _mm_mul_ps(dist, xn);

      __m128 xnyn = _mm_mul_ps(xn, yn);
      __m128 _2xn2 = _mm_add_ps(xn2, xn2);
      __m128 _2xnyn = _mm_add_ps(xnyn, xnyn);
      __m128 _2yn2 = _mm_add_ps(yn2, yn2);

      __m128 dist_yn = _mm_mul_ps(dist, yn);

      __m128 r2_2xn2 = _mm_add_ps(r2, _2xn2);
      __m128 r2_2yn2 = _mm_add_ps(r2, _2yn2);

      __m128 k3_2xnyn = _mm_mul_ps(*k3_ptr, _2xnyn);
      __m128 k3_r2_2yn2 = _mm_mul_ps(*k3_ptr, r2_2yn2);

      __m128 k4_r2_2xn2 = _mm_mul_ps(*k4_ptr, r2_2xn2);
      __m128 k4_2xnyn = _mm_mul_ps(*k4_ptr, _2xnyn);

      __m128 xd = _mm_add_ps(dist_xn, k3_2xnyn);
      xd = _mm_add_ps(xd, k4_r2_2xn2);

      __m128 yd = _mm_add_ps(dist_yn, k3_r2_2yn2);
      yd = _mm_add_ps(yd, k4_2xnyn);

      __m128 fx_xd = _mm_mul_ps(*fx_ptr, xd);
      __m128 fy_yd = _mm_mul_ps(*fy_ptr, yd);

      __m128 ux = _mm_add_ps(fx_xd, *px_ptr);
      __m128 uy = _mm_add_ps(fy_yd, *py_ptr);

      _mm_storeu_ps(&ux_f[0], ux);
      _mm_storeu_ps(&uy_f[0], uy);

      // Store results in pointcloud
      for (unsigned int i = 0; i < 4; i++) {
        if (ux_f[i] >= float(color->cols) || (ux_f[i]) < 0.0 || point[i].z == 0 || std::isnan(ux_f[i])) {
          point[i].r = 0;
          point[i].g = 0;
          point[i].b = 0;

          result.points[depth->cols * rows + cols + i].r = point[i].r;
          result.points[depth->cols * rows + cols + i].g = point[i].g;
          result.points[depth->cols * rows + cols + i].b = point[i].b;
          continue;
        }  // if point[i] projects out of rgb image, go to next point[i].

        if (uy_f[i] >= float(color->rows) || uy_f[i] < 0.0 || std::isnan(uy_f[i])) {
          point[i].r = 0;
          point[i].g = 0;
          point[i].b = 0;

          result.points[depth->cols * rows + cols + i].r = point[i].r;
          result.points[depth->cols * rows + cols + i].g = point[i].g;
          result.points[depth->cols * rows + cols + i].b = point[i].b;

          continue;
        }  // if point[i] projects out of rgb image, go to next point[i].

        cv::Vec3b rgb;
        rgb = color->at<cv::Vec3b>(int(uy_f[i]), int(ux_f[i]));

        point[i].r = rgb[0];
        point[i].g = rgb[1];
        point[i].b = rgb[2];

        result.points[depth->cols * rows + cols + i].r = point[i].r;
        result.points[depth->cols * rows + cols + i].g = point[i].g;
        result.points[depth->cols * rows + cols + i].b = point[i].b;
      }
    }
  }

  //    cv::imshow("ColorMap",*color);
  //    cv::waitKey(10);
}

void apply_calibration::GenerateDepthMapNoProjection(const cv::Mat* depth, cv::Mat& result) {
// SSE Implementation of depth generation
#pragma omp parallel for
  for (int rows = 0; rows < depth->rows; rows++) {
    for (int cols = 0; cols < depth->cols; cols = cols + 4) {
      float depth_sample[4];
      float ex[4];
      float sample_interpol[4];

      for (unsigned int i = 0; i < 4; i++) {
        depth_sample[i] = depth->at<float>(rows, cols + i);
        ex[i] = exp_lookup_[unsigned(depth_sample[i] * 1000.)];
        sample_interpol[i] = lattice_.at<float>(rows, cols + i);
      }

      // Project uv,depth to 3D space, project to rgb, using scale TREL and
      // intrinsics. make a color lookup (maybe an interpolation).

      float depth_sample_[]
          __attribute__((aligned(16))) = {depth_sample[0], depth_sample[1], depth_sample[2], depth_sample[3]};
      float sample_int[] __attribute__((aligned(16))) = {sample_interpol[0], sample_interpol[1], sample_interpol[2],
                                                         sample_interpol[3]};
      float ex_[] __attribute__((aligned(16))) = {ex[0], ex[1], ex[2], ex[3]};

      __m128* depth_ptr = (__m128*)depth_sample_;
      __m128* ex_ptr = (__m128*)ex_;
      __m128* sample_int_ptr = (__m128*)sample_int;

      __m128 invDepth = _mm_rcp_ps(*depth_ptr);
      __m128 delta = _mm_mul_ps(*ex_ptr, *sample_int_ptr);
      __m128 depth_measure = _mm_rcp_ps(_mm_add_ps(invDepth, delta));

      float z[4];

      _mm_storeu_ps(&z[0], depth_measure);

      for (int i = 0; i < 4; i++) {
        if (depth_sample[i] == 0.0) {
          result.at<ushort>(int(rows), int(cols + i)) = ushort(z[i] * 1000.);
          continue;
        } else {
          result.at<ushort>(int(rows), int(cols + i)) = ushort(z[i] * 1000.);
        }
      }
    }
  }
}
